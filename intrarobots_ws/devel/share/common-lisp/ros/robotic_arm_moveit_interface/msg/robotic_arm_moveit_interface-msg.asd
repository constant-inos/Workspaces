
(cl:in-package :asdf)

(defsystem "robotic_arm_moveit_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "go_to_pose_command" :depends-on ("_package_go_to_pose_command"))
    (:file "_package_go_to_pose_command" :depends-on ("_package"))
    (:file "go_to_pose_confirmation" :depends-on ("_package_go_to_pose_confirmation"))
    (:file "_package_go_to_pose_confirmation" :depends-on ("_package"))
    (:file "move_gripper_command" :depends-on ("_package_move_gripper_command"))
    (:file "_package_move_gripper_command" :depends-on ("_package"))
    (:file "move_gripper_confirmation" :depends-on ("_package_move_gripper_confirmation"))
    (:file "_package_move_gripper_confirmation" :depends-on ("_package"))
  ))