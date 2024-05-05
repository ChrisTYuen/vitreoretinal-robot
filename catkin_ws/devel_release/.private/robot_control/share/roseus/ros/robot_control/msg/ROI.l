;; Auto-generated. Do not edit!


(when (boundp 'robot_control::ROI)
  (if (not (find-package "ROBOT_CONTROL"))
    (make-package "ROBOT_CONTROL"))
  (shadow 'ROI (find-package "ROBOT_CONTROL")))
(unless (find-package "ROBOT_CONTROL::ROI")
  (make-package "ROBOT_CONTROL::ROI"))

(in-package "ROS")
;;//! \htmlinclude ROI.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass robot_control::ROI
  :super ros::object
  :slots (_roi_image _roi_values ))

(defmethod robot_control::ROI
  (:init
   (&key
    ((:roi_image __roi_image) (instance sensor_msgs::Image :init))
    ((:roi_values __roi_values) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _roi_image __roi_image)
   (setq _roi_values __roi_values)
   self)
  (:roi_image
   (&rest __roi_image)
   (if (keywordp (car __roi_image))
       (send* _roi_image __roi_image)
     (progn
       (if __roi_image (setq _roi_image (car __roi_image)))
       _roi_image)))
  (:roi_values
   (&optional __roi_values)
   (if __roi_values (setq _roi_values __roi_values)) _roi_values)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _roi_image
    (send _roi_image :serialization-length)
    ;; float64[] _roi_values
    (* 8    (length _roi_values)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _roi_image
       (send _roi_image :serialize s)
     ;; float64[] _roi_values
     (write-long (length _roi_values) s)
     (dotimes (i (length _roi_values))
       (sys::poke (elt _roi_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _roi_image
     (send _roi_image :deserialize buf ptr-) (incf ptr- (send _roi_image :serialization-length))
   ;; float64[] _roi_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _roi_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _roi_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get robot_control::ROI :md5sum-) "4345e91a8d0d0f4b5b72c6369217bad9")
(setf (get robot_control::ROI :datatype-) "robot_control/ROI")
(setf (get robot_control::ROI :definition-)
      "sensor_msgs/Image roi_image

float64[] roi_values


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :robot_control/ROI "4345e91a8d0d0f4b5b72c6369217bad9")


