; Auto-generated. Do not edit!


(cl:in-package robotic_arm_moveit_interface-msg)


;//! \htmlinclude move_gripper_confirmation.msg.html

(cl:defclass <move_gripper_confirmation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform "")
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform "")
   (opening_percentage
    :reader opening_percentage
    :initarg :opening_percentage
    :type cl:float
    :initform 0.0))
)

(cl:defclass move_gripper_confirmation (<move_gripper_confirmation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_gripper_confirmation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_gripper_confirmation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotic_arm_moveit_interface-msg:<move_gripper_confirmation> is deprecated: use robotic_arm_moveit_interface-msg:move_gripper_confirmation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <move_gripper_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:header-val is deprecated.  Use robotic_arm_moveit_interface-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <move_gripper_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:robot_name-val is deprecated.  Use robotic_arm_moveit_interface-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <move_gripper_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:cmd-val is deprecated.  Use robotic_arm_moveit_interface-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'opening_percentage-val :lambda-list '(m))
(cl:defmethod opening_percentage-val ((m <move_gripper_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:opening_percentage-val is deprecated.  Use robotic_arm_moveit_interface-msg:opening_percentage instead.")
  (opening_percentage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_gripper_confirmation>) ostream)
  "Serializes a message object of type '<move_gripper_confirmation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'opening_percentage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_gripper_confirmation>) istream)
  "Deserializes a message object of type '<move_gripper_confirmation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'opening_percentage) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_gripper_confirmation>)))
  "Returns string type for a message object of type '<move_gripper_confirmation>"
  "robotic_arm_moveit_interface/move_gripper_confirmation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_gripper_confirmation)))
  "Returns string type for a message object of type 'move_gripper_confirmation"
  "robotic_arm_moveit_interface/move_gripper_confirmation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_gripper_confirmation>)))
  "Returns md5sum for a message object of type '<move_gripper_confirmation>"
  "88a8a98da02f94ae31efa1de1c3281a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_gripper_confirmation)))
  "Returns md5sum for a message object of type 'move_gripper_confirmation"
  "88a8a98da02f94ae31efa1de1c3281a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_gripper_confirmation>)))
  "Returns full string definition for message of type '<move_gripper_confirmation>"
  (cl:format cl:nil "Header header~%string robot_name~%string cmd~%float32 opening_percentage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_gripper_confirmation)))
  "Returns full string definition for message of type 'move_gripper_confirmation"
  (cl:format cl:nil "Header header~%string robot_name~%string cmd~%float32 opening_percentage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_gripper_confirmation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     4 (cl:length (cl:slot-value msg 'cmd))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_gripper_confirmation>))
  "Converts a ROS message object to a list"
  (cl:list 'move_gripper_confirmation
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':opening_percentage (opening_percentage msg))
))
