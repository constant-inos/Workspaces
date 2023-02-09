#!/usr/bin/env python

import rospy
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import Pose,PoseStamped
from sensor_msgs.msg import LaserScan, Range, Image, JointState, PointCloud2, PointField
from sensor_msgs import point_cloud2
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64,String,Header
from moveit_msgs.msg import MoveGroupActionResult
from actionlib_msgs.msg import GoalStatus

import socket, pickle
import numpy as np
import time
import math
import cv2
from cv_bridge import CvBridge
import random
from tf.transformations import quaternion_from_euler
# from custom_messages.msg import *
import open3d
import struct
import colorsys
from sklearn.cluster import DBSCAN
import time 


bridge = CvBridge()
exev_status_code = 0
exev_status_text = ''
ROBOT_NAME = 'panda'

pick_command_publisher = rospy.Publisher("/"+ROBOT_NAME+"/pick_command",Grasp,queue_size=10)

class Master:
    def __init__(self,robot_name=ROBOT_NAME):
        self.robot_name = robot_name
        self.seq = 0
        self.ptc1 = -1
        self.ptc2 = -1
        self.ptc = -1
        self.transformation = np.eye(4)
        self.i1 = 0
        self.i2 = 0

    def create_transformation(self):
        T = np.eye(4)
        rotation_matrix  = self.ptc1.get_rotation_matrix_from_xyz((3*np.pi/2, 0, 0))
        a = 0.84*2/np.sqrt(2)
        translation_vector = [0,-a,a]
        T[:3, :3] = rotation_matrix
        T[0:3,3] = translation_vector
        self.transformation = T

    def transformation_matrix(self,pose1,pose0=(0,0,0,0,0,0)):
        (x0,y0,z0,roll0,pitch0,yaw0) = pose0
        (x1,y1,z1,roll1,pitch1,yaw1) = pose1

        translation_vector = [x1-x0,y1-y0,z1-z0]
        rotation_matrix  = self.ptc1.get_rotation_matrix_from_xyz((pitch1-pitch0, roll1-roll0,  yaw1-yaw0))

        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[0:3,3] = translation_vector

        return T

    def combine_ptcs(self,ptc1,ptc2):
        t0 = time.time()

        print("Combining Point Clouds")
        self.create_transformation()
        ptc1.transform(self.transformation)

        points = list(ptc1.points) + list(ptc2.points)
        colors = list(ptc1.colors) + list(ptc2.colors)

        points.append((0,0,0))
        colors.append((0,0,0))

        print("Total Points",len(points))

        ptc = open3d.geometry.PointCloud()
        ptc.points = open3d.utility.Vector3dVector(np.array(points))
        ptc.colors = open3d.utility.Vector3dVector(np.array(colors)) 

        print("Exec time for combine_ptc:",time.time()-t0)
        open3d.visualization.draw_geometries([ptc])
        return ptc

    def read_camera1(self,msg):
        self.i1 += 1
        ptc1,reds,greens = self.pt2_to_o3d(msg,scan_by_color=True)
        self.ptc1 = ptc1
        print("Camera 1 readings:",self.i1,", Camera 2 readings:",self.i2)

    def read_camera2(self,msg):
        self.i2 += 1
        ptc2,reds,greens = self.pt2_to_o3d(msg,scan_by_color=True)
        self.ptc2 = ptc2

        if self.i1 > 1 and self.i2 > 1:
            print("Syka Blyat")
            comb_ptc = self.combine_ptcs(self.ptc1,self.ptc2)
            print("Syka Blyat 2")
            # self.get_red_objects(comb_ptc)
            o = self.cluster_objects_3d(comb_ptc)
            o = [ob.paint_uniform_color([random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]) for ob in o]

            for oi in o:
                pt = list(oi.points)
                c = pt[int(len(pt)//2)]
                print("Centroid of Object",c)

            # open3d.visualization.draw_geometries(o)
            # self.convex_hull(o[1])
            # open3d.visualization.draw_geometries([a,b])
            print(" ")

    def pt2_to_o3d(self,msg,scan_by_color=False):
        t0 = time.time()

        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        FIELDS_XYZRGB = FIELDS_XYZ + [
                PointField(name='b', offset=16, datatype=PointField.UINT8, count=1),
                PointField(name='g', offset=17, datatype=PointField.UINT8, count=1),
                PointField(name='r', offset=18, datatype=PointField.UINT8, count=1),
            ]
        
        msg.fields = FIELDS_XYZRGB

        points = point_cloud2.read_points(msg,skip_nans=True)

        xyz = []
        rgb = []
        red_objects_points = []
        red_objects_colors = []
        green_objects_points = []
        green_objects_colors = []

        for p in points:
            x,y,z,b,g,r = p      
            # print(z)
            xyz.append((x,y,z))
            rgb.append((r,g,b))

            if scan_by_color:
                if is_red(r,g,b): 
                    red_objects_points.append((x,y,z))
                    red_objects_colors.append((r,g,b))
                if is_green(r,g,b): 
                    green_objects_points.append((x,y,z))
                    green_objects_points.append((r,g,b))

        # CREATE AN OPEN3D POINT CLOUD OF THE WORLD
        open3d_cloud = open3d.geometry.PointCloud()
        if len(xyz)>0:
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0) 
            # open3d_cloud.estimate_normals()   
        else:
            print("Zero Points Found")

        
        # CREATE AN OPEN3D POINT CLOUD CONTAINING ALL THE RED POINTS
        reds = open3d.geometry.PointCloud()
        if len(red_objects_points) > 0 and scan_by_color:
            reds.points = open3d.utility.Vector3dVector(np.array(red_objects_points))
            reds.colors = open3d.utility.Vector3dVector(np.array(red_objects_colors))

        # CREATE AN OPEN3D POINT CLOUD CONTAINING ALL THE GREEN POINTS
        greens = open3d.geometry.PointCloud()
        if len(green_objects_points)>0 and scan_by_color:
            greens.points = open3d.utility.Vector3dVector(np.array(green_objects_points))
            greens.colors = open3d.utility.Vector3dVector(np.array(green_objects_colors))

        print("Exec time for pt2_to_o3d:",time.time()-t0)
        # open3d.visualization.draw_geometries([reds])
        return open3d_cloud, reds, greens

        # input: point cloud
        # output: list of cluster of point clouds

        cluster_ind = np.array( point_cloud.cluster_dbscan(eps=0.01,min_points=4) )
        num_clusters = cluster_ind.max() + 1
        all_points = np.array(point_cloud.points)
        all_colors = np.array(point_cloud.colors)
        all_normals = np.array(point_cloud.normals)

        has_normals = len(list(all_normals)) == len(list(all_points))
        has_colors = len(list(all_colors)) == len(list(all_points))

        clusters = []

        for i in range(num_clusters):
            a = np.ones(cluster_ind.shape)
            a[cluster_ind != i] = 0

            cluster = open3d.geometry.PointCloud()

            cluster_points = all_points[a==1]
            cluster.points = open3d.utility.Vector3dVector(cluster_points)

            if has_colors: 
                cluster_colors = all_colors[a==1]
                cluster.colors = open3d.utility.Vector3dVector(cluster_colors)
            
            if has_normals: 
                cluster_normals = all_normals[a==1]
                cluster.normals = open3d.utility.Vector3dVector(cluster_normals)

            clusters.append(cluster)
        
        return clusters

    def convex_hull(self,ptc):
        # pcl = mesh.sample_points_poisson_disk(number_of_points=2000)
        hull, _ = ptc.compute_convex_hull()
        hull_ls = open3d.geometry.LineSet.create_from_triangle_mesh(hull)
        hull_ls.paint_uniform_color((1, 0, 0))
        open3d.visualization.draw_geometries([ptc, hull_ls])


def create_pose_msg(pose_goal_raw):
    quat = quaternion_from_euler(pose_goal_raw[3], pose_goal_raw[4], pose_goal_raw[5])
    pose = Pose()
    pose.position.x = pose_goal_raw[0]
    pose.position.y = pose_goal_raw[1]
    pose.position.z = pose_goal_raw[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


        

def read_camera(msg):
    C=0.025/14.0
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #print("Capturing image...")
    # cv_image = rotate_image(cv_image, 180)
    # cv_image = cv_image[140:270, 160:320]

    # cv_image = cv2.resize(stc=cv_image, dsize=())
    cv2.imwrite('/home/karagk/Workspaces/temp/chessboard.jpg',cv_image)
    cv2.imshow('Image',cv_image)
    cv2.waitKey(5)


def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result



def main():
    rospy.init_node('robot_state_publisher') #this is an existing topic

    rospy.spin()

COLOR_IMAGE_TOPIC = '/camera/color/image_raw'
POINT_CLOUD_TOPIC1 = '/camera/depth/points'
POINT_CLOUD_TOPIC2 = '/camera2/depth/points'

sub = rospy.Subscriber(COLOR_IMAGE_TOPIC, Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image

if __name__ == "__main__":
    main()
