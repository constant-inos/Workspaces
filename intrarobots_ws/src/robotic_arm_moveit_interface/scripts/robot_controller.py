#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Range, Image, JointState
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

bridge = CvBridge()
exev_status_code = 0
exev_status_text = ''
ROBOT_NAME = 'panda'

go_to_pose_command_publisher = rospy.Publisher("/"+ROBOT_NAME+"/go_to_pose_command",GoToPoseCommand,queue_size=1)
move_gripper_command_publisher = rospy.Publisher("/"+ROBOT_NAME+"/move_gripper_command",MoveGripperCommand,queue_size=1)

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
    pose_euler = [x,y,z,0.0,np.pi,theta]
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
    
    return

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
    
    return

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

q = []
busy = False
standby_pose = (0.5,0.0,0.4)

def read_camera(msg):
    open_gripper()

    global standby_pose

    C=0.025/14.0
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #print("Capturing image...")
    objects = get_objects_position(cv_image)

    busy = True
    for o in objects:
        success = get_object(o[0],o[1])

    busy = False
    sub.unregister()
    sub2.unregister()
    main()

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


from moveit_msgs.msg import MoveGroupActionResult
from actionlib_msgs.msg import GoalStatus
import time

def main():
    rospy.init_node('robot_state_publisher') #this is an existing topic
    
    rospy.spin()

sub = rospy.Subscriber("/"+ROBOT_NAME+"/camera_link_optical/image_raw", Image, read_camera) # Camera sensor, Image is the data type sensor_msgs/Image
sub2 = rospy.Subscriber("/"+ROBOT_NAME+"/move_group/result", MoveGroupActionResult, execution_status)

if __name__ == "__main__":
    main()
