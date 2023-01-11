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
  :slots (_header _robot_name _command_id _success ))

(defmethod custom_messages::GoToPoseConfirmation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:command_id __command_id) "")
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _command_id (string __command_id))
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
  (:command_id
   (&optional __command_id)
   (if __command_id (setq _command_id __command_id)) _command_id)
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
    ;; string _command_id
    4 (length _command_id)
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
     ;; string _command_id
       (write-long (length _command_id) s) (princ _command_id s)
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
   ;; string _command_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get custom_messages::GoToPoseConfirmation :md5sum-) "d1e861f7b9dd4217596bf13b98116564")
(setf (get custom_messages::GoToPoseConfirmation :datatype-) "custom_messages/GoToPoseConfirmation")
(setf (get custom_messages::GoToPoseConfirmation :definition-)
      "Header header
string robot_name
string command_id
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



(provide :custom_messages/GoToPoseConfirmation "d1e861f7b9dd4217596bf13b98116564")


