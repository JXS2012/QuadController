; Auto-generated. Do not edit!


(cl:in-package quadrotorTestControl-srv)


;//! \htmlinclude getTargetPosition-request.msg.html

(cl:defclass <getTargetPosition-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getTargetPosition-request (<getTargetPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getTargetPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getTargetPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotorTestControl-srv:<getTargetPosition-request> is deprecated: use quadrotorTestControl-srv:getTargetPosition-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getTargetPosition-request>) ostream)
  "Serializes a message object of type '<getTargetPosition-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getTargetPosition-request>) istream)
  "Deserializes a message object of type '<getTargetPosition-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getTargetPosition-request>)))
  "Returns string type for a service object of type '<getTargetPosition-request>"
  "quadrotorTestControl/getTargetPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTargetPosition-request)))
  "Returns string type for a service object of type 'getTargetPosition-request"
  "quadrotorTestControl/getTargetPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getTargetPosition-request>)))
  "Returns md5sum for a message object of type '<getTargetPosition-request>"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getTargetPosition-request)))
  "Returns md5sum for a message object of type 'getTargetPosition-request"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getTargetPosition-request>)))
  "Returns full string definition for message of type '<getTargetPosition-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getTargetPosition-request)))
  "Returns full string definition for message of type 'getTargetPosition-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getTargetPosition-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getTargetPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getTargetPosition-request
))
;//! \htmlinclude getTargetPosition-response.msg.html

(cl:defclass <getTargetPosition-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass getTargetPosition-response (<getTargetPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getTargetPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getTargetPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotorTestControl-srv:<getTargetPosition-response> is deprecated: use quadrotorTestControl-srv:getTargetPosition-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <getTargetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotorTestControl-srv:x-val is deprecated.  Use quadrotorTestControl-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <getTargetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotorTestControl-srv:y-val is deprecated.  Use quadrotorTestControl-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <getTargetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotorTestControl-srv:z-val is deprecated.  Use quadrotorTestControl-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getTargetPosition-response>) ostream)
  "Serializes a message object of type '<getTargetPosition-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getTargetPosition-response>) istream)
  "Deserializes a message object of type '<getTargetPosition-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getTargetPosition-response>)))
  "Returns string type for a service object of type '<getTargetPosition-response>"
  "quadrotorTestControl/getTargetPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTargetPosition-response)))
  "Returns string type for a service object of type 'getTargetPosition-response"
  "quadrotorTestControl/getTargetPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getTargetPosition-response>)))
  "Returns md5sum for a message object of type '<getTargetPosition-response>"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getTargetPosition-response)))
  "Returns md5sum for a message object of type 'getTargetPosition-response"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getTargetPosition-response>)))
  "Returns full string definition for message of type '<getTargetPosition-response>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getTargetPosition-response)))
  "Returns full string definition for message of type 'getTargetPosition-response"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getTargetPosition-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getTargetPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getTargetPosition-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getTargetPosition)))
  'getTargetPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getTargetPosition)))
  'getTargetPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTargetPosition)))
  "Returns string type for a service object of type '<getTargetPosition>"
  "quadrotorTestControl/getTargetPosition")