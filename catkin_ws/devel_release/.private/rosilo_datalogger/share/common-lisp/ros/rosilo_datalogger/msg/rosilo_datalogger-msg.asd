
(cl:in-package :asdf)

(defsystem "rosilo_datalogger-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddValueMsg" :depends-on ("_package_AddValueMsg"))
    (:file "_package_AddValueMsg" :depends-on ("_package"))
  ))