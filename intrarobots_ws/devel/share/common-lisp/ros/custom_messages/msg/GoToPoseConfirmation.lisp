; Auto-generated. Do not edit!


(cl:in-package custom_messages-msg)


;//! \htmlinclude GoToPoseConfirmation.msg.html

(cl:defclass <GoToPoseConfirmation> (roslisp-msg-protocol:ros-message)
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
   (command_id
    :reader command_id
    :initarg :command_id
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GoToPoseConfirmation (<GoToPoseConfirmation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToPoseConfirmation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToPoseConfirmation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_messages-msg:<GoToPoseConfirmation> is deprecated: use custom_messages-msg:GoToPoseConfirmation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GoToPoseConfirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-msg:header-val is deprecated.  Use custom_messages-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <GoToPoseConfirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-msg:robot_name-val is deprecated.  Use custom_messages-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'command_id-val :lambda-list '(m))
(cl:defmethod command_id-val ((m <GoToPoseConfirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-msg:command_id-val is deprecated.  Use custom_messages-msg:command_id instead.")
  (command_id m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GoToPoseConfirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-msg:success-val is deprecated.  Use custom_messages-msg:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToPoseConfirmation>) ostream)
  "Serializes a message object of type '<GoToPoseConfirmation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToPoseConfirmation>) istream)
  "Deserializes a message object of type '<GoToPoseConfirmation>"
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
      (cl:setf (cl:slot-value msg 'command_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToPoseConfirmation>)))
  "Returns string type for a message object of type '<GoToPoseConfirmation>"
  "custom_messages/GoToPoseConfirmation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPoseConfirmation)))
  "Returns string type for a message object of type 'GoToPoseConfirmation"
  "custom_messages/GoToPoseConfirmation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToPoseConfirmation>)))
  "Returns md5sum for a message object of type '<GoToPoseConfirmation>"
  "d1e861f7b9dd4217596bf13b98116564")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToPoseConfirmation)))
  "Returns md5sum for a message object of type 'GoToPoseConfirmation"
  "d1e861f7b9dd4217596bf13b98116564")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToPoseConfirmation>)))
  "Returns full string definition for message of type '<GoToPoseConfirmation>"
  (cl:format cl:nil "Header header~%string robot_name~%string command_id~%bool success~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToPoseConfirmation)))
  "Returns full string definition for message of type 'GoToPoseConfirmation"
  (cl:format cl:nil "Header header~%string robot_name~%string command_id~%bool success~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToPoseConfirmation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     4 (cl:length (cl:slot-value msg 'command_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToPoseConfirmation>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToPoseConfirmation
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':command_id (command_id msg))
    (cl:cons ':success (success msg))
))
