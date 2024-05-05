; Auto-generated. Do not edit!


(cl:in-package rosilo_datalogger-srv)


;//! \htmlinclude Save-request.msg.html

(cl:defclass <Save-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass Save-request (<Save-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Save-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Save-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosilo_datalogger-srv:<Save-request> is deprecated: use rosilo_datalogger-srv:Save-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <Save-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosilo_datalogger-srv:filename-val is deprecated.  Use rosilo_datalogger-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Save-request>) ostream)
  "Serializes a message object of type '<Save-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Save-request>) istream)
  "Deserializes a message object of type '<Save-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Save-request>)))
  "Returns string type for a service object of type '<Save-request>"
  "rosilo_datalogger/SaveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Save-request)))
  "Returns string type for a service object of type 'Save-request"
  "rosilo_datalogger/SaveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Save-request>)))
  "Returns md5sum for a message object of type '<Save-request>"
  "86a64704c03f5415ea7ff59624fa3411")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Save-request)))
  "Returns md5sum for a message object of type 'Save-request"
  "86a64704c03f5415ea7ff59624fa3411")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Save-request>)))
  "Returns full string definition for message of type '<Save-request>"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Save-request)))
  "Returns full string definition for message of type 'Save-request"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Save-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Save-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Save-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude Save-response.msg.html

(cl:defclass <Save-response> (roslisp-msg-protocol:ros-message)
  ((worked
    :reader worked
    :initarg :worked
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Save-response (<Save-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Save-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Save-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosilo_datalogger-srv:<Save-response> is deprecated: use rosilo_datalogger-srv:Save-response instead.")))

(cl:ensure-generic-function 'worked-val :lambda-list '(m))
(cl:defmethod worked-val ((m <Save-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosilo_datalogger-srv:worked-val is deprecated.  Use rosilo_datalogger-srv:worked instead.")
  (worked m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Save-response>) ostream)
  "Serializes a message object of type '<Save-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'worked) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Save-response>) istream)
  "Deserializes a message object of type '<Save-response>"
    (cl:setf (cl:slot-value msg 'worked) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Save-response>)))
  "Returns string type for a service object of type '<Save-response>"
  "rosilo_datalogger/SaveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Save-response)))
  "Returns string type for a service object of type 'Save-response"
  "rosilo_datalogger/SaveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Save-response>)))
  "Returns md5sum for a message object of type '<Save-response>"
  "86a64704c03f5415ea7ff59624fa3411")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Save-response)))
  "Returns md5sum for a message object of type 'Save-response"
  "86a64704c03f5415ea7ff59624fa3411")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Save-response>)))
  "Returns full string definition for message of type '<Save-response>"
  (cl:format cl:nil "bool worked~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Save-response)))
  "Returns full string definition for message of type 'Save-response"
  (cl:format cl:nil "bool worked~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Save-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Save-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Save-response
    (cl:cons ':worked (worked msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Save)))
  'Save-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Save)))
  'Save-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Save)))
  "Returns string type for a service object of type '<Save>"
  "rosilo_datalogger/Save")