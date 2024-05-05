;; Auto-generated. Do not edit!


(when (boundp 'rosilo_datalogger::AddValueMsg)
  (if (not (find-package "ROSILO_DATALOGGER"))
    (make-package "ROSILO_DATALOGGER"))
  (shadow 'AddValueMsg (find-package "ROSILO_DATALOGGER")))
(unless (find-package "ROSILO_DATALOGGER::ADDVALUEMSG")
  (make-package "ROSILO_DATALOGGER::ADDVALUEMSG"))

(in-package "ROS")
;;//! \htmlinclude AddValueMsg.msg.html


(defclass rosilo_datalogger::AddValueMsg
  :super ros::object
  :slots (_name _value _strvalue ))

(defmethod rosilo_datalogger::AddValueMsg
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:strvalue __strvalue) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value __value)
   (setq _strvalue (string __strvalue))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:strvalue
   (&optional __strvalue)
   (if __strvalue (setq _strvalue __strvalue)) _strvalue)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64[] _value
    (* 8    (length _value)) 4
    ;; string _strvalue
    4 (length _strvalue)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64[] _value
     (write-long (length _value) s)
     (dotimes (i (length _value))
       (sys::poke (elt _value i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _strvalue
       (write-long (length _strvalue) s) (princ _strvalue s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _value
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _value (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _value i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _strvalue
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _strvalue (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get rosilo_datalogger::AddValueMsg :md5sum-) "d17a07a93f3e7d7722ef08216d8e72f9")
(setf (get rosilo_datalogger::AddValueMsg :datatype-) "rosilo_datalogger/AddValueMsg")
(setf (get rosilo_datalogger::AddValueMsg :definition-)
      "string name
float64[] value
string strvalue

")



(provide :rosilo_datalogger/AddValueMsg "d17a07a93f3e7d7722ef08216d8e72f9")


