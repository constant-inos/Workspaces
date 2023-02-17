roslaunch interbotix_xslocobot_nav xslocobot_nav_sim_2.launch robot_model:=locobot_wx200

roslaunch interbotix_xslocobot_moveit_interface xslocobot_moveit_interface.launch robot_model:=locobot_wx200 use_gazebo:=true use_camera:=true use_python_interface:=true

rosservice call /gazebo/unpause_physics

# Control Velocities from keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/locobot/cmd_vel