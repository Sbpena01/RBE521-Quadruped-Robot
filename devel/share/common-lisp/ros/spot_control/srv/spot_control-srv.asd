
(cl:in-package :asdf)

(defsystem "spot_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "jointControl" :depends-on ("_package_jointControl"))
    (:file "_package_jointControl" :depends-on ("_package"))
    (:file "multiplier" :depends-on ("_package_multiplier"))
    (:file "_package_multiplier" :depends-on ("_package"))
  ))