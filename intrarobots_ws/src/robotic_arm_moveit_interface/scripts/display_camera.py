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
import open3d
import struct
import colorsys
from sklearn.cluster import DBSCAN
import time 

# from ultralytics import YOLO

bridge = CvBridge()
exev_status_code = 0
exev_status_text = ''
ROBOT_NAME = 'panda'

pick_command_publisher = rospy.Publisher("/"+ROBOT_NAME+"/pick_command",Grasp,queue_size=10)

# weights = 'yolov8n.pt'
# model = YOLO(weights)


def read_img_and_ptc(img_msg,ptc_msg):
    "Time Synchronized Callback worked"
    print(f"IMG: {img_msg.header.seq}   {img_msg.header.stamp.secs} {img_msg.header.stamp.nsecs}")
    print(f"PTC: {ptc_msg.header.seq}   {ptc_msg.header.stamp.secs} {ptc_msg.header.stamp.nsecs}")

def read_camera(msg):
    C=0.025/14.0
    print("Capturing image...")
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    print(f"Image shape: {cv_image.shape}")
    # cv_image = rotate_image(cv_image, 180)
    # cv_image = cv_image[140:270, 160:320]

    # cv_image = inference(cv_image)
    cv2.imshow('Image',cv_image)
    cv2.waitKey(5) # milisecond

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

def inference(img):

        (X0,Y0) = img.shape[:2]

        objects = []
        t0 = time.time()
        pred = model.predict(img)
        n_boxes = len(pred[0].boxes)
        [X,Y] = [int(x) for x in pred[0].boxes.orig_shape]

        for i in range(n_boxes):
            box = pred[0].boxes[i]
            conf = float(box.conf)
            [x1,y1,x2,y2] = [int(x) for x in box.xyxy[0]]
            det_class = int(box.cls)
            x1,x2,y1,y2 = int(x1/X*X0),int(x2/X*X0),int(y1/Y*Y0),int(y2/Y*Y0)
            img = cv2.rectangle(img, (x1,y1), (x2,y2), (255, 0, 0), 2)

        print(img.shape)
        print(x1,y1,x2,y2)
        cv2.imshow('Image',img)
        cv2.waitKey(5)


def main():
    # rospy.init_node('robot_state_publisher') #this is an existing topic
    rospy.sleep(3)
    rospy.spin()

COLOR_IMAGE_TOPIC = '/camera/color/image_raw'
POINT_CLOUD_TOPIC = '/camera/depth/points'
POINT_CLOUD_TOPIC2 = '/camera2/depth/points'

# sub = rospy.Subscriber(COLOR_IMAGE_TOPIC, Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image

# camera_subscriber = rospy.Subscriber(COLOR_IMAGE_TOPIC, Image)
# pointcloud2_subscriber = rospy.Subscriber(POINT_CLOUD_TOPIC, PointCloud2)

import message_filters
rospy.init_node('robot_state_publisher')
camera_subscriber = message_filters.Subscriber(COLOR_IMAGE_TOPIC, Image)
pointcloud2_subscriber = message_filters.Subscriber(POINT_CLOUD_TOPIC, PointCloud2)

ts = message_filters.TimeSynchronizer([camera_subscriber,pointcloud2_subscriber],1)
ts.registerCallback(read_img_and_ptc)

if __name__ == "__main__":
    rospy.spin()
