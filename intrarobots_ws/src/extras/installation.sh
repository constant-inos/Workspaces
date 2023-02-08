# INSTALL ROS NOETIC ON UBUNTU 20
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# INSTALL GAZEBO, RVIZ ?
sudo apt install gazebo11

# INSTALL REQUIRED PACKAGES
# sudo apt install ros-noetic-slam-gmapping

# CREATE CATKIN WS ?

# GIT CLONE CURRENT CODE BRANCH

# BUILD WORKSPACE

#INSTALL PYTHON LIBRARIES

## ON EVERY GIT CLONE


# build catkin workspace 
USER_NAME=$(whoami) && bash /home/$USER_NAME/Workspaces/intrarobots_ws/src/extras/create_packages.sh 
USER_NAME=$(whoami) && sudo bash /home/$USER_NAME/Workspaces/intrarobots_ws/src/extras/update_gazebo_models.sh 