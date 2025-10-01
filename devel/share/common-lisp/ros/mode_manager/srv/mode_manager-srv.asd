
(cl:in-package :asdf)

(defsystem "mode_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ModeRequest" :depends-on ("_package_ModeRequest"))
    (:file "_package_ModeRequest" :depends-on ("_package"))
  ))