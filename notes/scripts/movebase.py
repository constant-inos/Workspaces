#!/usr/bin/env python

import rospy
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node("move_base_commander")

client = actionlib.SimpleActionClient('/locobot/move_base',MoveBaseAction)
client.wait_for_server()

# Creates a new goal with the MoveBaseGoal constructor
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
# Move 0.5 meters forward along the x axis of the "map" coordinate frame 
goal.target_pose.pose.position.x = 0.0
# No rotation of the mobile base frame w.r.t. map frame
goal.target_pose.pose.orientation.w = 5.0



client.send_goal(goal)
print("Goal")
print(goal)
wait = client.wait_for_result()
print(wait)