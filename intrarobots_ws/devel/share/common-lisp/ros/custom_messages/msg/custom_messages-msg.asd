
(cl:in-package :asdf)

(defsystem "custom_messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GoToPoseCommand" :depends-on ("_package_GoToPoseCommand"))
    (:file "_package_GoToPoseCommand" :depends-on ("_package"))
    (:file "GoToPoseConfirmation" :depends-on ("_package_GoToPoseConfirmation"))
    (:file "_package_GoToPoseConfirmation" :depends-on ("_package"))
    (:file "MoveGripperCommand" :depends-on ("_package_MoveGripperCommand"))
    (:file "_package_MoveGripperCommand" :depends-on ("_package"))
    (:file "MoveGripperConfirmation" :depends-on ("_package_MoveGripperConfirmation"))
    (:file "_package_MoveGripperConfirmation" :depends-on ("_package"))
  ))