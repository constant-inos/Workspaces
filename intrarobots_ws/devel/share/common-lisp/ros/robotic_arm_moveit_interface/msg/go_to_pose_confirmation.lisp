; Auto-generated. Do not edit!


(cl:in-package robotic_arm_moveit_interface-msg)


;//! \htmlinclude go_to_pose_confirmation.msg.html

(cl:defclass <go_to_pose_confirmation> (roslisp-msg-protocol:ros-message)
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
   (target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass go_to_pose_confirmation (<go_to_pose_confirmation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <go_to_pose_confirmation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'go_to_pose_confirmation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotic_arm_moveit_interface-msg:<go_to_pose_confirmation> is deprecated: use robotic_arm_moveit_interface-msg:go_to_pose_confirmation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <go_to_pose_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:header-val is deprecated.  Use robotic_arm_moveit_interface-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <go_to_pose_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:robot_name-val is deprecated.  Use robotic_arm_moveit_interface-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <go_to_pose_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:target_pose-val is deprecated.  Use robotic_arm_moveit_interface-msg:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <go_to_pose_confirmation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm_moveit_interface-msg:success-val is deprecated.  Use robotic_arm_moveit_interface-msg:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <go_to_pose_confirmation>) ostream)
  "Serializes a message object of type '<go_to_pose_confirmation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <go_to_pose_confirmation>) istream)
  "Deserializes a message object of type '<go_to_pose_confirmation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<go_to_pose_confirmation>)))
  "Returns string type for a message object of type '<go_to_pose_confirmation>"
  "robotic_arm_moveit_interface/go_to_pose_confirmation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'go_to_pose_confirmation)))
  "Returns string type for a message object of type 'go_to_pose_confirmation"
  "robotic_arm_moveit_interface/go_to_pose_confirmation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<go_to_pose_confirmation>)))
  "Returns md5sum for a message object of type '<go_to_pose_confirmation>"
  "d8653ffd7c3f3bfa775e8baaf947f5e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'go_to_pose_confirmation)))
  "Returns md5sum for a message object of type 'go_to_pose_confirmation"
  "d8653ffd7c3f3bfa775e8baaf947f5e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<go_to_pose_confirmation>)))
  "Returns full string definition for message of type '<go_to_pose_confirmation>"
  (cl:format cl:nil "Header header~%string robot_name~%geometry_msgs/Pose target_pose~%bool success~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'go_to_pose_confirmation)))
  "Returns full string definition for message of type 'go_to_pose_confirmation"
  (cl:format cl:nil "Header header~%string robot_name~%geometry_msgs/Pose target_pose~%bool success~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <go_to_pose_confirmation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <go_to_pose_confirmation>))
  "Converts a ROS message object to a list"
  (cl:list 'go_to_pose_confirmation
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':success (success msg))
))
