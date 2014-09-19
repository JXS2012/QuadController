
(cl:in-package :asdf)

(defsystem "quadrotorTestControl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getTargetVelocity" :depends-on ("_package_getTargetVelocity"))
    (:file "_package_getTargetVelocity" :depends-on ("_package"))
    (:file "getTargetPosition" :depends-on ("_package_getTargetPosition"))
    (:file "_package_getTargetPosition" :depends-on ("_package"))
  ))