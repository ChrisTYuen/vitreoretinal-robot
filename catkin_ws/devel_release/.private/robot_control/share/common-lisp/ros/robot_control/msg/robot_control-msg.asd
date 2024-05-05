
(cl:in-package :asdf)

(defsystem "robot_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ImgShowMsg" :depends-on ("_package_ImgShowMsg"))
    (:file "_package_ImgShowMsg" :depends-on ("_package"))
    (:file "ROI" :depends-on ("_package_ROI"))
    (:file "_package_ROI" :depends-on ("_package"))
  ))