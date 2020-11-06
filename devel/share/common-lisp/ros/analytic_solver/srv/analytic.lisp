; Auto-generated. Do not edit!


(cl:in-package analytic_solver-srv)


;//! \htmlinclude analytic-request.msg.html

(cl:defclass <analytic-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass analytic-request (<analytic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <analytic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'analytic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name analytic_solver-srv:<analytic-request> is deprecated: use analytic_solver-srv:analytic-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <analytic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader analytic_solver-srv:a-val is deprecated.  Use analytic_solver-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <analytic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader analytic_solver-srv:b-val is deprecated.  Use analytic_solver-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <analytic-request>) ostream)
  "Serializes a message object of type '<analytic-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <analytic-request>) istream)
  "Deserializes a message object of type '<analytic-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<analytic-request>)))
  "Returns string type for a service object of type '<analytic-request>"
  "analytic_solver/analyticRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'analytic-request)))
  "Returns string type for a service object of type 'analytic-request"
  "analytic_solver/analyticRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<analytic-request>)))
  "Returns md5sum for a message object of type '<analytic-request>"
  "6a2e34150c00229791cc89ff309fff21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'analytic-request)))
  "Returns md5sum for a message object of type 'analytic-request"
  "6a2e34150c00229791cc89ff309fff21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<analytic-request>)))
  "Returns full string definition for message of type '<analytic-request>"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'analytic-request)))
  "Returns full string definition for message of type 'analytic-request"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <analytic-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <analytic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'analytic-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude analytic-response.msg.html

(cl:defclass <analytic-response> (roslisp-msg-protocol:ros-message)
  ((sum
    :reader sum
    :initarg :sum
    :type cl:integer
    :initform 0))
)

(cl:defclass analytic-response (<analytic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <analytic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'analytic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name analytic_solver-srv:<analytic-response> is deprecated: use analytic_solver-srv:analytic-response instead.")))

(cl:ensure-generic-function 'sum-val :lambda-list '(m))
(cl:defmethod sum-val ((m <analytic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader analytic_solver-srv:sum-val is deprecated.  Use analytic_solver-srv:sum instead.")
  (sum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <analytic-response>) ostream)
  "Serializes a message object of type '<analytic-response>"
  (cl:let* ((signed (cl:slot-value msg 'sum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <analytic-response>) istream)
  "Deserializes a message object of type '<analytic-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sum) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<analytic-response>)))
  "Returns string type for a service object of type '<analytic-response>"
  "analytic_solver/analyticResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'analytic-response)))
  "Returns string type for a service object of type 'analytic-response"
  "analytic_solver/analyticResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<analytic-response>)))
  "Returns md5sum for a message object of type '<analytic-response>"
  "6a2e34150c00229791cc89ff309fff21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'analytic-response)))
  "Returns md5sum for a message object of type 'analytic-response"
  "6a2e34150c00229791cc89ff309fff21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<analytic-response>)))
  "Returns full string definition for message of type '<analytic-response>"
  (cl:format cl:nil "int64 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'analytic-response)))
  "Returns full string definition for message of type 'analytic-response"
  (cl:format cl:nil "int64 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <analytic-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <analytic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'analytic-response
    (cl:cons ':sum (sum msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'analytic)))
  'analytic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'analytic)))
  'analytic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'analytic)))
  "Returns string type for a service object of type '<analytic>"
  "analytic_solver/analytic")