;; Auto-generated. Do not edit!


(when (boundp 'robotic_arm_moveit_interface::go_to_pose_confirmation)
  (if (not (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE"))
    (make-package "ROBOTIC_ARM_MOVEIT_INTERFACE"))
  (shadow 'go_to_pose_confirmation (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE")))
(unless (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE::GO_TO_POSE_CONFIRMATION")
  (make-package "ROBOTIC_ARM_MOVEIT_INTERFACE::GO_TO_POSE_CONFIRMATION"))

(in-package "ROS")
;;//! \htmlinclude go_to_pose_confirmation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robotic_arm_moveit_interface::go_to_pose_confirmation
  :super ros::object
  :slots (_header _robot_name _target_pose _success ))

(defmethod robotic_arm_moveit_interface::go_to_pose_confirmation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _target_pose __target_pose)
   (setq _success __success)
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
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ;; bool _success
    1
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
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
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
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robotic_arm_moveit_interface::go_to_pose_confirmation :md5sum-) "d8653ffd7c3f3bfa775e8baaf947f5e1")
(setf (get robotic_arm_moveit_interface::go_to_pose_confirmation :datatype-) "robotic_arm_moveit_interface/go_to_pose_confirmation")
(setf (get robotic_arm_moveit_interface::go_to_pose_confirmation :definition-)
      "Header header
string robot_name
geometry_msgs/Pose target_pose
bool success

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



(provide :robotic_arm_moveit_interface/go_to_pose_confirmation "d8653ffd7c3f3bfa775e8baaf947f5e1")


