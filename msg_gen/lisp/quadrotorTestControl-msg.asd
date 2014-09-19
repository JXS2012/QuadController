
(cl:in-package :asdf)

(defsystem "quadrotorTestControl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Point3D" :depends-on ("_package_Point3D"))
    (:file "_package_Point3D" :depends-on ("_package"))
  ))