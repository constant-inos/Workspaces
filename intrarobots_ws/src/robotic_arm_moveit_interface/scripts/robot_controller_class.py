#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Range, Image, JointState, PointCloud2, PointField
from sensor_msgs import point_cloud2
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Float64
import socket, pickle
import numpy as np
import time
import math
import cv2
from cv_bridge import CvBridge
import random
from tf.transformations import quaternion_from_euler
from custom_messages.msg import *
import open3d
import struct
import colorsys
from sklearn.cluster import DBSCAN
import time 

bridge = CvBridge()
exev_status_code = 0
exev_status_text = ''
ROBOT_NAME = 'panda'

go_to_pose_command_publisher = rospy.Publisher("/"+ROBOT_NAME+"/go_to_pose_command",GoToPoseCommand,queue_size=10)
move_gripper_command_publisher = rospy.Publisher("/"+ROBOT_NAME+"/move_gripper_command",MoveGripperCommand,queue_size=10)

class Master:
    def __init__(self):
        self.ptc1 = -1
        self.ptc2 = -1
        self.ptc = -1
        self.i1 = 0
        self.i2 = 0

    def create_transformation(self):
        T = np.eye(4)
        rotation_matrix  = self.ptc1.get_rotation_matrix_from_xyz((3*np.pi/2, 0, 0))
        a = 0.84*2/np.sqrt(2)
        translation_vector = [0,-a,a]
        T[:3, :3] = rotation_matrix
        T[0:3,3] = translation_vector
        
        return T

    def align_axes_system(self,ptc):
        # T = np.eye(4) 
        
        # T[:3, :3] = rotation_matrix
        # T[0:3,3] = translation_vector

        R = ptc.get_rotation_matrix_from_xyz((-np.pi/4, 0, 0))
        ptc.rotate(R,center=(0, 0, 0))

        R = ptc.get_rotation_matrix_from_xyz((0, np.pi/2, 0))
        ptc.rotate(R,center=(0, 0, 0))

        R = ptc.get_rotation_matrix_from_xyz((-np.pi/2, 0, 0))
        ptc.rotate(R,center=(0, 0, 0))

        translation_vector = [-0.84,0.54,0.8]
        ptc.translate(translation_vector)

        return ptc
             
    def add_axis_display(self,ptc):
        # black at center
        # red   at x axis
        # green at y axis
        # blue  at z axis

        points = list(ptc.points)
        colors = list(ptc.colors)

        points.append((0,0,0))
        colors.append((0,0,0))

        points.append((0.1,0,0))
        colors.append((1,0,0))

        points.append((0,0.1,0))
        colors.append((0,1,0))

        points.append((0,0,0.1))
        colors.append((0,0,1))

        ptc = open3d.geometry.PointCloud()
        ptc.points = open3d.utility.Vector3dVector(np.array(points))
        ptc.colors = open3d.utility.Vector3dVector(np.array(colors)) 

        return ptc

    def scan_by_height(self,ptc,h=0.116):
        points = list(ptc.points)
        colors = list(ptc.colors)

        p = []
        c = []

        for i in range(len(points)):
            if points[i][2] > h:
                p.append(points[i])
                c.append(colors[i])

        ptc = open3d.geometry.PointCloud()
        ptc.points = open3d.utility.Vector3dVector(np.array(p))
        ptc.colors = open3d.utility.Vector3dVector(np.array(c)) 

        return ptc

    def combine_ptcs(self,ptc1,ptc2):
        t0 = time.time()

        print("Combining Point Clouds")
        ptc1.transform(self.create_transformation())

        points = list(ptc1.points) + list(ptc2.points)
        colors = list(ptc1.colors) + list(ptc2.colors)
        
        ptc = open3d.geometry.PointCloud()
        ptc.points = open3d.utility.Vector3dVector(np.array(points))
        ptc.colors = open3d.utility.Vector3dVector(np.array(colors)) 

        ptc = self.align_axes_system(ptc)
        # ptc_ = self.add_axis_display(ptc)

        print("Exec time for combine_ptc:",time.time()-t0)
        # open3d.visualization.draw_geometries([ptc])
        return ptc

    def get_red_objects(self,ptc):

        t0 = time.time()

        points = list(ptc.points)
        colors = list(ptc.colors)

        red_points = []
        red_colors = []
        for i in range(len(colors)):
            r,g,b = colors[i]
            r,g,b = 255*r,255*g,255*b
            if is_red(r,g,b): 
                red_points.append(points[i])
                red_colors.append(colors[i])

        ptc = open3d.geometry.PointCloud()
        ptc.points = open3d.utility.Vector3dVector(np.array(red_points))
        ptc.colors = open3d.utility.Vector3dVector(np.array(red_colors))

        print("Exec time for get_red_objects:",time.time()-t0)
        open3d.visualization.draw_geometries([ptc])
        
        return ptc

    def read_camera1(self,msg):

        self.i1 += 1
        ptc1,reds,greens = self.pt2_to_o3d(msg,scan_by_color=True)
        self.ptc1 = reds
        print("Camera 1 readings:",self.i1,", Camera 2 readings:",self.i2)

    def read_camera2(self,msg):
        self.i2 += 1
        ptc2,reds,greens = self.pt2_to_o3d(msg,scan_by_color=True)
        self.ptc2 = reds

        # self.test_motion_planning()

        if self.i1 > 1 and self.i2 > 1:
            print("Cyka Blyat")
            comb_ptc = self.combine_ptcs(self.ptc1,self.ptc2)
            comb_ptc = self.scan_by_height(comb_ptc)
            print("Cyka Blyat 2")
            # self.get_red_objects(comb_ptc)
            o = self.cluster_objects_3d(comb_ptc)
            o = [ob.paint_uniform_color([random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]) for ob in o]

            # open3d.visualization.draw_geometries(o)
            # self.convex_hull(o[1])

            # open3d.visualization.draw_geometries([a,b])
            obj = o[1]
            p1,p2 = self.get_gripper_orientation(obj)


            # print(p1,p2)
            # points = list(obj.points) + [p1,p2]
            # colors = list(obj.colors) + [(1,0,0),(1,0,0)]

            # ptc = open3d.geometry.PointCloud()
            # ptc.points = open3d.utility.Vector3dVector(np.array(points))
            # ptc.colors = open3d.utility.Vector3dVector(np.array(colors)) 
            # open3d.visualization.draw_geometries([ptc])

            # print(" ")

            c = (np.array(p1) + np.array(p2)) / 2
            s = self.get_object(c,'red')
            return


    def get_gripper_orientation(self,object_points):
        object_points.estimate_normals()
        object_points.orient_normals_consistent_tangent_plane(k=3)

        points = object_points.points
        colors = object_points.colors
        normals = object_points.normals

        a = open3d.geometry.PointCloud()
        a.points = normals
        a.normals = points
        a.colors = colors

        clusters_of_normals = self.cluster_objects_3d(a)
        P,N = [],[]

        for c in clusters_of_normals:
            points = c.normals
            colors = c.colors
            normals = c.points
            
            m = len(list(points))
            p = list(points)[int(m/2)]
            n = list(normals)[int(m/2)]

            P.append(p)
            N.append(n)
            print("Point:", p)
            print("Normal:", n)
        
        d_max = -1
        p1 = -1
        p2 = -1
        for i in range(len(P)):
            for j in range(len(P)):
                d = math.dist(N[i],N[j]) 
                if math.dist(N[i],N[j]) > d_max:
                    d_max = d
                    p1 = P[i]+0.001
                    p2 = P[j]+0.001

        

        return p1,p2
            
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

    def cluster_objects_3d(self,point_cloud):
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

    def test_motion_planning(self):
        standby_pose = (0.5,0.0,0.4)
        move_to_specified_pose(standby_pose)
        
        move_to_specified_pose((0.5,0.3,0.4))


    def get_object(self,coords,color):

        x,y = coords[0],coords[1]

        if x<0: phi = np.pi - np.arctan(-y/x)
        else: phi = np.arctan(y/x)
        pose = (x,y,0.25)

        if color == 'red':
            bucket = (0.35,-0.35,0.25)
        else:
            bucket = (-0.35,-0.35,0.25)
        move_to_specified_pose(standby_pose)

        open_gripper()

        lower = (pose[0],pose[1],0.14)
        move_to_specified_pose(lower)

        close_gripper()

        move_to_specified_pose(pose)

        move_to_specified_pose(bucket)

        open_gripper()

        move_to_specified_pose(standby_pose)

    


def create_pose_msg(pose_goal_raw):
    quat = quaternion_from_euler(pose_goal_raw[3], pose_goal_raw[4], pose_goal_raw[5])
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pose_goal_raw[0]
    pose.position.y = pose_goal_raw[1]
    pose.position.z = pose_goal_raw[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def move_to_specified_pose(value):

    print("Moving to Specified Pose")

    x,y,z = value
    theta = get_gripper_angle(x,y)
    pose_euler = [x,y,z,0,np.pi,theta]
    pose_quat = create_pose_msg(pose_euler)
    
    msg = GoToPoseCommand()
    msg.robot_name = ROBOT_NAME
    msg.target_pose = pose_quat
    uid = str(type(msg))+'-'+str(msg.header.seq)
    msg.command_id = uid
    go_to_pose_command_publisher.publish(msg)

    result = await_command_completion(uid)
    
    if result == 'SUCCESS':
        print("Moved to specified pose successfully")
    
    return (result == 'SUCCESS')

def control_motor_angles(value):
    arr=[]
    arr.append("control_motor_angles")
    arr.append(value)
    # send_socket_msg(arr)

def open_gripper():
    print("Opening Gripper")
    msg = MoveGripperCommand()
    msg.robot_name = ROBOT_NAME
    msg.command = 'open'
    uid = str(type(msg))+'-'+str(msg.header.seq)
    msg.command_id = uid
    move_gripper_command_publisher.publish(msg)

    result = await_command_completion(uid)
    
    if result == 'SUCCESS':
        print("Gripper Opened Successfully")
    
    return (result == 'SUCCESS')

def close_gripper():
    print("Closing Gripper")
    msg = MoveGripperCommand()
    msg.robot_name = 'panda'
    msg.command = 'close'
    uid = str(type(msg))+'-'+str(msg.header.seq)
    msg.command_id = uid
    move_gripper_command_publisher.publish(msg)
    
    result = await_command_completion(uid)
    
    if result == 'SUCCESS':
        print("Gripper Closed Successfully")
        return True
    else:
        return False

    return 

def await_command_completion(uid):
    rospy.set_param('/command/uid',uid)
    rospy.set_param('/command/received',False)
    rospy.set_param('/command/completed',False)
    rospy.set_param('/command/success',False)

    while True:
        received = rospy.get_param('/command/received')
        completed = rospy.get_param('/command/completed')
        success = rospy.get_param('/command/success')

        if not received:
            continue
        if completed and success:
            return('SUCCESS')
        if completed and not success:
            return('FAILED')
        
def ranges_to_polar(ranges,object_r=0.025,angle_inc=0.031733):
    ranges = list(ranges)
    cut_ranges = []
    index_cut_ranges = []
    for i in range(3):
        min_range = min(ranges)
        ind = ranges.index(min_range)
        cut_ranges.append(min_range)
        index_cut_ranges.append(ind)
        ranges[ind] = float('inf')

    d = np.mean(cut_ranges[:3]) + object_r
    theta = np.mean(index_cut_ranges)*angle_inc
    return (d,theta)

def get_objects_position(cv_image):
    # hsv format: hue, saturation, value (brightness)
    img_hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # lower red mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    red_mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    # upper red mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    red_mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    # join my masks
    red_mask = red_mask1 + red_mask0
    # lower green mask (hue 40-70)
    lower_green = np.array([40,40,40])
    upper_green = np.array([70,255,255])
    green_mask = cv2.inRange(img_hsv, lower_green, upper_green)
    # set my output img to zero everywhere except my mask
    output_img = cv_image.copy()
    only_reds = cv2.bitwise_and(output_img, output_img, mask=red_mask)
    only_greens = cv2.bitwise_and(output_img, output_img, mask=green_mask)

    filtered = only_reds + only_greens

    objects = []

    K_reds,cr = count_objects(only_reds)
    K_greens,cg = count_objects(only_greens)


    print("Red objects found:",K_reds)
    print("Green objects found:",K_greens)
    if K_reds > 0:
        gray = cv2.cvtColor(only_reds, cv2.COLOR_BGR2GRAY)
        red_points = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if gray[i,j]:
                    red_points.append(np.array([i,j]))
        red_points = np.float32(red_points)
        criteria = (cv2.TERM_CRITERIA_EPS, 10, 1.0)
        ret, label, center = cv2.kmeans(red_points, K_reds, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        for c in list(center):
            objects.append((c.round(),'red'))

    if K_greens > 0:
        gray = cv2.cvtColor(only_greens, cv2.COLOR_BGR2GRAY)
        green_points = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if gray[i,j]:
                    green_points.append(np.array([i,j]))
        green_points = np.float32(green_points)
        criteria = (cv2.TERM_CRITERIA_EPS, 10, 1.0)
        ret, label, center = cv2.kmeans(green_points, K_greens, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        for c in list(center):
            objects.append((c.round(),'green'))

    return objects

def count_objects(image):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image
    blur = cv2.GaussianBlur(gray, (11, 11), 0)
    canny = cv2.Canny(blur, 30, 150, 3)
    dilated = cv2.dilate(canny, (1, 1), iterations=0)
    (cnt, hierarchy) = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return len(cnt),cnt

def pixel2coords(px,py,H=480,W=640):
    C=0.025/14.0

    Xcv = px * C
    Ycv = py * C

    x = -Xcv + C*H/2
    y = -Ycv + (0.54 + C*W/2.0)

    # small perception corrections
    x = x + 0.025*x/0.4
    y = y - 0.025*(0.54-y)/0.25

    return x,y

def is_red(r,g,b):
    # if r>1.0 or g>1.0 or b>1.0:
    r,g,b = r/255.0,b/255.0,g/255.0
    (h,s,v) = colorsys.rgb_to_hsv(r,g,b)
    (h,s,v) = (h*180,s*255,v*255)

    c1 = (h<10) and (s>50) and (v>50)
    c2 = (h>170 and h<180) and (s>200) and (v>50)
    
    return (c1 or c2)
    return c1

def is_green(r,g,b):
    if r>1.0 or g>1.0 or b>1.0:
        r,g,b = r/255.0,b/255.0,g/255.0
    (h,s,v) = colorsys.rgb_to_hsv(r,g,b)
    (h,s,v) = (h*180,s*255,v*255)

    return ((h>=40 and h<=70) and (s>40) and (v>40))
        
q = []
busy = False
standby_pose = (0.5,0.0,0.4)

def read_camera(msg):
    open_gripper()

    global standby_pose

    C=0.025/14.0
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #print("Capturing image...")
    print(cv_image.shape)

    objects = get_objects_position(cv_image)
    
    busy = True
    for o in objects:
        print(o)
        success = get_object(o[0],o[1])

    busy = False

def get_object(obj_pixels,color):

    x,y = pixel2coords(obj_pixels[0],obj_pixels[1])

    if x<0: phi = np.pi - np.arctan(-y/x)
    else: phi = np.arctan(y/x)
    pose = (x,y,0.25)

    if color == 'red':
        bucket = (0.35,-0.35,0.25)
    else:
        bucket = (-0.35,-0.35,0.25)
    move_to_specified_pose(standby_pose)

    move_to_specified_pose(pose)

    open_gripper()

    lower = (pose[0],pose[1],0.14)
    move_to_specified_pose(lower)

    close_gripper()

    move_to_specified_pose(pose)

    move_to_specified_pose(bucket)

    open_gripper()

    move_to_specified_pose(standby_pose)



def get_gripper_angle(x,y):
    if (x<0 and y>0):
        theta = np.pi - np.arctan(-y/x)
    elif (x<0 and y<0):
        theta = -np.pi + np.arctan(y/x)
    else:
        if x == 0: return np.arctan(np.inf)
        theta = np.arctan(y/x)
    return theta

def execution_status(msg):
    global exev_status_text, exec_status_code
    exev_status_text, exec_status_code = msg.status.text, msg.status.status
    print('exec_status')
    return exev_status_text, exec_status_code

def read_point_cloud2(msg):
    height = msg.height # how many rows in the data
    width = msg.width # how many points in a row
    point_step = msg.point_step # how many bytes in a point
    row_step = msg.row_step # how many bytes in a row
    data = msg.data
    is_bigendian = msg.is_bigendian

    image = np.zeros((height,width,3))
    i = 0

    for row in range(height):
        for point in range(width):
            offset = row * row_step + point * point_step

            x_bytes = data[offset:offset+4]
            x = struct.unpack('f',x_bytes)

            y_bytes = data[offset+4:offset+8]
            y = struct.unpack('f',y_bytes)
            
            z_bytes = data[offset+8:offset+16]
            z = struct.unpack('ff',z_bytes)

            b_bytes = data[offset+16:offset+17]
            b = struct.unpack('B',b_bytes)
            b = int(b[0])
            
            g_bytes = data[offset+17:offset+18]
            g = struct.unpack('B',g_bytes)
            g = int(g[0])

            r_bytes = data[offset+18:offset+19]
            r = struct.unpack('B',r_bytes)
            r = int(r[0]) 

            image[row,point,0] = b
            image[row,point,1] = g
            image[row,point,2] = r
            # print(b,g,r)
            i +=1
    return image




def get_point_cloud(msg):

    # image = read_point_cloud2(msg)
    o3d_cloud = pt2_to_o3d(msg,scan_by_color=True)
    # open3d.visualization.draw_geometries([reds,greens])
    return
    
    image = image.astype(np.uint8)
    objects = get_objects_position(image)

    for o in objects:
        print(o)
        success = get_object(o[0],o[1])

from moveit_msgs.msg import MoveGroupActionResult
from actionlib_msgs.msg import GoalStatus
import time

master = Master()

def main():
    rospy.init_node('robot_state_publisher') #this is an existing topic
    
    rospy.spin()

COLOR_IMAGE_TOPIC = '/camera/color/image_raw'
POINT_CLOUD_TOPIC1 = '/camera/depth/points'
POINT_CLOUD_TOPIC2 = '/camera2/depth/points'

point_cloud_subscriber1 = rospy.Subscriber(POINT_CLOUD_TOPIC1, PointCloud2, master.read_camera1)
point_cloud_subscriber2 = rospy.Subscriber(POINT_CLOUD_TOPIC2, PointCloud2, master.read_camera2)
# sub = rospy.Subscriber(COLOR_IMAGE_TOPIC, Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
# sub2 = rospy.Subscriber("/"+ROBOT_NAME+"/move_group/result", MoveGroupActionResult, execution_status)

if __name__ == "__main__":
    main()
