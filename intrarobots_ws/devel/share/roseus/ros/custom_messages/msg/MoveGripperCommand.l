;; Auto-generated. Do not edit!


(when (boundp 'custom_messages::MoveGripperCommand)
  (if (not (find-package "CUSTOM_MESSAGES"))
    (make-package "CUSTOM_MESSAGES"))
  (shadow 'MoveGripperCommand (find-package "CUSTOM_MESSAGES")))
(unless (find-package "CUSTOM_MESSAGES::MOVEGRIPPERCOMMAND")
  (make-package "CUSTOM_MESSAGES::MOVEGRIPPERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MoveGripperCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_messages::MoveGripperCommand
  :super ros::object
  :slots (_header _robot_name _command_id _command ))

(defmethod custom_messages::MoveGripperCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:command_id __command_id) "")
    ((:command __command) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _command_id (string __command_id))
   (setq _command (string __command))
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
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; string _command_id
    4 (length _command_id)
    ;; string _command
    4 (length _command)
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
     ;; string _command
       (write-long (length _command) s) (princ _command s)
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
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get custom_messages::MoveGripperCommand :md5sum-) "01641dfa5597b5e4124d01564db602c3")
(setf (get custom_messages::MoveGripperCommand :datatype-) "custom_messages/MoveGripperCommand")
(setf (get custom_messages::MoveGripperCommand :definition-)
      "Header header
string robot_name
string command_id
string command

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



(provide :custom_messages/MoveGripperCommand "01641dfa5597b5e4124d01564db602c3")


