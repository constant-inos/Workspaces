;; Auto-generated. Do not edit!


(when (boundp 'custom_messages::GoToPoseConfirmation)
  (if (not (find-package "CUSTOM_MESSAGES"))
    (make-package "CUSTOM_MESSAGES"))
  (shadow 'GoToPoseConfirmation (find-package "CUSTOM_MESSAGES")))
(unless (find-package "CUSTOM_MESSAGES::GOTOPOSECONFIRMATION")
  (make-package "CUSTOM_MESSAGES::GOTOPOSECONFIRMATION"))

(in-package "ROS")
;;//! \htmlinclude GoToPoseConfirmation.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_messages::GoToPoseConfirmation
  :super ros::object
  :slots (_header _robot_name _command_seq _success ))

(defmethod custom_messages::GoToPoseConfirmation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:command_seq __command_seq) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _command_seq (round __command_seq))
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
  (:command_seq
   (&optional __command_seq)
   (if __command_seq (setq _command_seq __command_seq)) _command_seq)
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
    ;; uint32 _command_seq
    4
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
     ;; uint32 _command_seq
       (write-long _command_seq s)
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
   ;; uint32 _command_seq
     (setq _command_seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get custom_messages::GoToPoseConfirmation :md5sum-) "bf37a5a5bd1d5d54d0cd95b0a2f2076c")
(setf (get custom_messages::GoToPoseConfirmation :datatype-) "custom_messages/GoToPoseConfirmation")
(setf (get custom_messages::GoToPoseConfirmation :definition-)
      "Header header
string robot_name
uint32 command_seq
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

")



(provide :custom_messages/GoToPoseConfirmation "bf37a5a5bd1d5d54d0cd95b0a2f2076c")


