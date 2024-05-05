;; Auto-generated. Do not edit!


(when (boundp 'rosilo_polaris_vega::ToolsPoseArray)
  (if (not (find-package "ROSILO_POLARIS_VEGA"))
    (make-package "ROSILO_POLARIS_VEGA"))
  (shadow 'ToolsPoseArray (find-package "ROSILO_POLARIS_VEGA")))
(unless (find-package "ROSILO_POLARIS_VEGA::TOOLSPOSEARRAY")
  (make-package "ROSILO_POLARIS_VEGA::TOOLSPOSEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ToolsPoseArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rosilo_polaris_vega::ToolsPoseArray
  :super ros::object
  :slots (_header _frame_numbers _port_handles _statuses _poses ))

(defmethod rosilo_polaris_vega::ToolsPoseArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:frame_numbers __frame_numbers) (make-array 0 :initial-element 0 :element-type :integer))
    ((:port_handles __port_handles) (make-array 0 :initial-element 0 :element-type :integer))
    ((:statuses __statuses) (make-array 0 :initial-element 0 :element-type :integer))
    ((:poses __poses) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _frame_numbers __frame_numbers)
   (setq _port_handles __port_handles)
   (setq _statuses __statuses)
   (setq _poses __poses)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:frame_numbers
   (&optional __frame_numbers)
   (if __frame_numbers (setq _frame_numbers __frame_numbers)) _frame_numbers)
  (:port_handles
   (&optional __port_handles)
   (if __port_handles (setq _port_handles __port_handles)) _port_handles)
  (:statuses
   (&optional __statuses)
   (if __statuses (setq _statuses __statuses)) _statuses)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64[] _frame_numbers
    (* 8    (length _frame_numbers)) 4
    ;; int32[] _port_handles
    (* 4    (length _port_handles)) 4
    ;; int32[] _statuses
    (* 4    (length _statuses)) 4
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64[] _frame_numbers
     (write-long (length _frame_numbers) s)
     (dotimes (i (length _frame_numbers))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _frame_numbers i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _frame_numbers i)) (= (length ((elt _frame_numbers i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _frame_numbers i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _frame_numbers i) . bv) 1) -1) s))
             ((and (class (elt _frame_numbers i)) (= (length ((elt _frame_numbers i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _frame_numbers i) . bv) 0) s)
              (write-long (if (>= (elt _frame_numbers i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _frame_numbers i) s)(write-long (if (>= (elt _frame_numbers i) 0) 0 #xffffffff) s)))
       )
     ;; int32[] _port_handles
     (write-long (length _port_handles) s)
     (dotimes (i (length _port_handles))
       (write-long (elt _port_handles i) s)
       )
     ;; int32[] _statuses
     (write-long (length _statuses) s)
     (dotimes (i (length _statuses))
       (write-long (elt _statuses i) s)
       )
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64[] _frame_numbers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frame_numbers (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _frame_numbers i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _frame_numbers i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; int32[] _port_handles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _port_handles (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _port_handles i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _statuses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _statuses (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _statuses i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rosilo_polaris_vega::ToolsPoseArray :md5sum-) "297b84503018526d42525e774a191c54")
(setf (get rosilo_polaris_vega::ToolsPoseArray :datatype-) "rosilo_polaris_vega/ToolsPoseArray")
(setf (get rosilo_polaris_vega::ToolsPoseArray :definition-)
      "Header header

int64[] frame_numbers
int32[] port_handles
int32[] statuses
geometry_msgs/Pose[] poses

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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :rosilo_polaris_vega/ToolsPoseArray "297b84503018526d42525e774a191c54")


