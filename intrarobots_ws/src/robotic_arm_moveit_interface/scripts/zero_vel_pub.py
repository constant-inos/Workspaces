#!/usr/bin/env python

import rospy
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import Pose,PoseStamped,Twist,Vector3
from sensor_msgs.msg import LaserScan, Range, Image, JointState, PointCloud2, PointField
from sensor_msgs import point_cloud2
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String,Header
from std_msgs.msg import Float64

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

ROBOT_NAME = 'locobot'

CMD_VEL_TOPIC = "/"+ROBOT_NAME+"/cmd_vel"


cmd_vel_publisher = rospy.Publisher(CMD_VEL_TOPIC,Twist,queue_size=10)

linear = Vector3()
angular = Vector3()
vel = Twist()

linear.x,linear.y,linear.z = 0, 0, 0
angular.x,angular.y,angular.z = 0, 0, 0
vel.linear, vel.angular = linear, angular

def main():
    rospy.init_node('zero_vel_pub') #this is an existing topic
    cmd_vel_publisher.publish(vel)
    print(3)
    rospy.spin()


if __name__ == "__main__":
    while True:
        main()
