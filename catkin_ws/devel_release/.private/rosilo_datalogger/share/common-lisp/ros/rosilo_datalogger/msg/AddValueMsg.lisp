; Auto-generated. Do not edit!


(cl:in-package rosilo_datalogger-msg)


;//! \htmlinclude AddValueMsg.msg.html

(cl:defclass <AddValueMsg> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (strvalue
    :reader strvalue
    :initarg :strvalue
    :type cl:string
    :initform ""))
)

(cl:defclass AddValueMsg (<AddValueMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddValueMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddValueMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosilo_datalogger-msg:<AddValueMsg> is deprecated: use rosilo_datalogger-msg:AddValueMsg instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <AddValueMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosilo_datalogger-msg:name-val is deprecated.  Use rosilo_datalogger-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <AddValueMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosilo_datalogger-msg:value-val is deprecated.  Use rosilo_datalogger-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'strvalue-val :lambda-list '(m))
(cl:defmethod strvalue-val ((m <AddValueMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosilo_datalogger-msg:strvalue-val is deprecated.  Use rosilo_datalogger-msg:strvalue instead.")
  (strvalue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddValueMsg>) ostream)
  "Serializes a message object of type '<AddValueMsg>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'value))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'strvalue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'strvalue))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddValueMsg>) istream)
  "Deserializes a message object of type '<AddValueMsg>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'value) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'value)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'strvalue) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'strvalue) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddValueMsg>)))
  "Returns string type for a message object of type '<AddValueMsg>"
  "rosilo_datalogger/AddValueMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddValueMsg)))
  "Returns string type for a message object of type 'AddValueMsg"
  "rosilo_datalogger/AddValueMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddValueMsg>)))
  "Returns md5sum for a message object of type '<AddValueMsg>"
  "d17a07a93f3e7d7722ef08216d8e72f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddValueMsg)))
  "Returns md5sum for a message object of type 'AddValueMsg"
  "d17a07a93f3e7d7722ef08216d8e72f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddValueMsg>)))
  "Returns full string definition for message of type '<AddValueMsg>"
  (cl:format cl:nil "string name~%float64[] value~%string strvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddValueMsg)))
  "Returns full string definition for message of type 'AddValueMsg"
  (cl:format cl:nil "string name~%float64[] value~%string strvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddValueMsg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:length (cl:slot-value msg 'strvalue))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddValueMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AddValueMsg
    (cl:cons ':name (name msg))
    (cl:cons ':value (value msg))
    (cl:cons ':strvalue (strvalue msg))
))
