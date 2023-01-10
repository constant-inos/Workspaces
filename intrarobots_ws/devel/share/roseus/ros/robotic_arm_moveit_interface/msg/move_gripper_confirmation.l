;; Auto-generated. Do not edit!


(when (boundp 'robotic_arm_moveit_interface::move_gripper_confirmation)
  (if (not (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE"))
    (make-package "ROBOTIC_ARM_MOVEIT_INTERFACE"))
  (shadow 'move_gripper_confirmation (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE")))
(unless (find-package "ROBOTIC_ARM_MOVEIT_INTERFACE::MOVE_GRIPPER_CONFIRMATION")
  (make-package "ROBOTIC_ARM_MOVEIT_INTERFACE::MOVE_GRIPPER_CONFIRMATION"))

(in-package "ROS")
;;//! \htmlinclude move_gripper_confirmation.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robotic_arm_moveit_interface::move_gripper_confirmation
  :super ros::object
  :slots (_header _robot_name _cmd _opening_percentage ))

(defmethod robotic_arm_moveit_interface::move_gripper_confirmation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:cmd __cmd) "")
    ((:opening_percentage __opening_percentage) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _cmd (string __cmd))
   (setq _opening_percentage (float __opening_percentage))
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
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:opening_percentage
   (&optional __opening_percentage)
   (if __opening_percentage (setq _opening_percentage __opening_percentage)) _opening_percentage)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; string _cmd
    4 (length _cmd)
    ;; float32 _opening_percentage
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;; float32 _opening_percentage
       (sys::poke _opening_percentage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _opening_percentage
     (setq _opening_percentage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotic_arm_moveit_interface::move_gripper_confirmation :md5sum-) "88a8a98da02f94ae31efa1de1c3281a7")
(setf (get robotic_arm_moveit_interface::move_gripper_confirmation :datatype-) "robotic_arm_moveit_interface/move_gripper_confirmation")
(setf (get robotic_arm_moveit_interface::move_gripper_confirmation :definition-)
      "Header header
string robot_name
string cmd
float32 opening_percentage

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

")



(provide :robotic_arm_moveit_interface/move_gripper_confirmation "88a8a98da02f94ae31efa1de1c3281a7")


