
(cl:in-package :asdf)

(defsystem "mcuserial_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RequestParam" :depends-on ("_package_RequestParam"))
    (:file "_package_RequestParam" :depends-on ("_package"))
  ))