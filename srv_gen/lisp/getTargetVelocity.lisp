; Auto-generated. Do not edit!


(cl:in-package quadrotorTestControl-srv)


;//! \htmlinclude getTargetVelocity-request.msg.html

(cl:defclass <getTargetVelocity-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getTargetVelocity-request (<getTargetVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getTargetVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getTargetVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotorTestControl-srv:<getTargetVelocity-request> is deprecated: use quadrotorTestControl-srv:getTargetVelocity-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getTargetVelocity-request>) ostream)
  "Serializes a message object of type '<getTargetVelocity-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getTargetVelocity-request>) istream)
  "Deserializes a message object of type '<getTargetVelocity-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getTargetVelocity-request>)))
  "Returns string type for a service object of type '<getTargetVelocity-request>"
  "quadrotorTestControl/getTargetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTargetVelocity-request)))
  "Returns string type for a service object of type 'getTargetVelocity-request"
  "quadrotorTestControl/getTargetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getTargetVelocity-request>)))
  "Returns md5sum for a message object of type '<getTargetVelocity-request>"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getTargetVelocity-request)))
  "Returns md5sum for a message object of type 'getTargetVelocity-request"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getTargetVelocity-request>)))
  "Returns full string definition for message of type '<getTargetVelocity-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getTargetVelocity-request)))
  "Returns full string definition for message of type 'getTargetVelocity-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getTargetVelocity-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getTargetVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getTargetVelocity-request
))
;//! \htmlinclude getTargetVelocity-response.msg.html

(cl:defclass <getTargetVelocity-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass getTargetVelocity-response (<getTargetVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getTargetVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getTargetVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotorTestControl-srv:<getTargetVelocity-response> is deprecated: use quadrotorTestControl-srv:getTargetVelocity-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <getTargetVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotorTestControl-srv:x-val is deprecated.  Use quadrotorTestControl-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <getTargetVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotorTestControl-srv:y-val is deprecated.  Use quadrotorTestControl-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <getTargetVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotorTestControl-srv:z-val is deprecated.  Use quadrotorTestControl-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getTargetVelocity-response>) ostream)
  "Serializes a message object of type '<getTargetVelocity-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getTargetVelocity-response>) istream)
  "Deserializes a message object of type '<getTargetVelocity-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getTargetVelocity-response>)))
  "Returns string type for a service object of type '<getTargetVelocity-response>"
  "quadrotorTestControl/getTargetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTargetVelocity-response)))
  "Returns string type for a service object of type 'getTargetVelocity-response"
  "quadrotorTestControl/getTargetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getTargetVelocity-response>)))
  "Returns md5sum for a message object of type '<getTargetVelocity-response>"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getTargetVelocity-response)))
  "Returns md5sum for a message object of type 'getTargetVelocity-response"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getTargetVelocity-response>)))
  "Returns full string definition for message of type '<getTargetVelocity-response>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getTargetVelocity-response)))
  "Returns full string definition for message of type 'getTargetVelocity-response"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getTargetVelocity-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getTargetVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getTargetVelocity-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getTargetVelocity)))
  'getTargetVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getTargetVelocity)))
  'getTargetVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getTargetVelocity)))
  "Returns string type for a service object of type '<getTargetVelocity>"
  "quadrotorTestControl/getTargetVelocity")