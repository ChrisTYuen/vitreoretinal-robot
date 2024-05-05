
(cl:in-package :asdf)

(defsystem "rosilo_polaris_vega-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ToolsPoseArray" :depends-on ("_package_ToolsPoseArray"))
    (:file "_package_ToolsPoseArray" :depends-on ("_package"))
  ))