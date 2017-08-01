
(cl:in-package :asdf)

(defsystem "mitya_teleop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Drive" :depends-on ("_package_Drive"))
    (:file "_package_Drive" :depends-on ("_package"))
    (:file "HeadPosition" :depends-on ("_package_HeadPosition"))
    (:file "_package_HeadPosition" :depends-on ("_package"))
  ))