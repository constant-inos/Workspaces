;; Auto-generated. Do not edit!


(when (boundp 'robotic_arm_moveit_interface::go_to_pose_command)
  (if (not (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE"))
    (make-package "ROBOTIC_ARM_MOVEIT_INTERFACE"))
  (shadow 'go_to_pose_command (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE")))
(unless (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE::GO_TO_POSE_COMMAND")
  (make-package "ROBOTIC_ARM_MOVEIT_INTERFACE::GO_TO_POSE_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude go_to_pose_command.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robotic_arm_moveit_interface::go_to_pose_command
  :super ros::object
  :slots (_header _robot_name _target_pose ))

(defmethod robotic_arm_moveit_interface::go_to_pose_command
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _target_pose __target_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;;
   self)
  )

(setf (get robotic_arm_moveit_interface::go_to_pose_command :md5sum-) "05eddb1fda3dba35a09c800d8d38c1ae")
(setf (get robotic_arm_moveit_interface::go_to_pose_command :datatype-) "robotic_arm_moveit_interface/go_to_pose_command")
(setf (get robotic_arm_moveit_interface::go_to_pose_command :definition-)
      "Header header
string robot_name
geometry_msgs/Pose target_pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :robotic_arm_moveit_interface/go_to_pose_command "05eddb1fda3dba35a09c800d8d38c1ae")


