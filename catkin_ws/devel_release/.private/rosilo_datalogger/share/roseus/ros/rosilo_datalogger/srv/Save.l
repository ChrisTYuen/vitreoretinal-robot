;; Auto-generated. Do not edit!


(when (boundp 'rosilo_datalogger::Save)
  (if (not (find-package "ROSILO_DATALOGGER"))
    (make-package "ROSILO_DATALOGGER"))
  (shadow 'Save (find-package "ROSILO_DATALOGGER")))
(unless (find-package "ROSILO_DATALOGGER::SAVE")
  (make-package "ROSILO_DATALOGGER::SAVE"))
(unless (find-package "ROSILO_DATALOGGER::SAVEREQUEST")
  (make-package "ROSILO_DATALOGGER::SAVEREQUEST"))
(unless (find-package "ROSILO_DATALOGGER::SAVERESPONSE")
  (make-package "ROSILO_DATALOGGER::SAVERESPONSE"))

(in-package "ROS")





(defclass rosilo_datalogger::SaveRequest
  :super ros::object
  :slots (_filename ))

(defmethod rosilo_datalogger::SaveRequest
  (:init
   (&key
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _filename (string __filename))
   self)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rosilo_datalogger::SaveResponse
  :super ros::object
  :slots (_worked ))

(defmethod rosilo_datalogger::SaveResponse
  (:init
   (&key
    ((:worked __worked) nil)
    )
   (send-super :init)
   (setq _worked __worked)
   self)
  (:worked
   (&optional (__worked :null))
   (if (not (eq __worked :null)) (setq _worked __worked)) _worked)
  (:serialization-length
   ()
   (+
    ;; bool _worked
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _worked
       (if _worked (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _worked
     (setq _worked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rosilo_datalogger::Save
  :super ros::object
  :slots ())

(setf (get rosilo_datalogger::Save :md5sum-) "86a64704c03f5415ea7ff59624fa3411")
(setf (get rosilo_datalogger::Save :datatype-) "rosilo_datalogger/Save")
(setf (get rosilo_datalogger::Save :request) rosilo_datalogger::SaveRequest)
(setf (get rosilo_datalogger::Save :response) rosilo_datalogger::SaveResponse)

(defmethod rosilo_datalogger::SaveRequest
  (:response () (instance rosilo_datalogger::SaveResponse :init)))

(setf (get rosilo_datalogger::SaveRequest :md5sum-) "86a64704c03f5415ea7ff59624fa3411")
(setf (get rosilo_datalogger::SaveRequest :datatype-) "rosilo_datalogger/SaveRequest")
(setf (get rosilo_datalogger::SaveRequest :definition-)
      "string filename
---
bool worked

")

(setf (get rosilo_datalogger::SaveResponse :md5sum-) "86a64704c03f5415ea7ff59624fa3411")
(setf (get rosilo_datalogger::SaveResponse :datatype-) "rosilo_datalogger/SaveResponse")
(setf (get rosilo_datalogger::SaveResponse :definition-)
      "string filename
---
bool worked

")



(provide :rosilo_datalogger/Save "86a64704c03f5415ea7ff59624fa3411")


