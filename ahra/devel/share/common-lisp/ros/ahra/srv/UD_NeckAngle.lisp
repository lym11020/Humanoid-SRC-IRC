; Auto-generated. Do not edit!


(cl:in-package ahra-srv)


;//! \htmlinclude UD_NeckAngle-request.msg.html

(cl:defclass <UD_NeckAngle-request> (roslisp-msg-protocol:ros-message)
  ((finish
    :reader finish
    :initarg :finish
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UD_NeckAngle-request (<UD_NeckAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UD_NeckAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UD_NeckAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<UD_NeckAngle-request> is deprecated: use ahra-srv:UD_NeckAngle-request instead.")))

(cl:ensure-generic-function 'finish-val :lambda-list '(m))
(cl:defmethod finish-val ((m <UD_NeckAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:finish-val is deprecated.  Use ahra-srv:finish instead.")
  (finish m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UD_NeckAngle-request>) ostream)
  "Serializes a message object of type '<UD_NeckAngle-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finish) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UD_NeckAngle-request>) istream)
  "Deserializes a message object of type '<UD_NeckAngle-request>"
    (cl:setf (cl:slot-value msg 'finish) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UD_NeckAngle-request>)))
  "Returns string type for a service object of type '<UD_NeckAngle-request>"
  "ahra/UD_NeckAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UD_NeckAngle-request)))
  "Returns string type for a service object of type 'UD_NeckAngle-request"
  "ahra/UD_NeckAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UD_NeckAngle-request>)))
  "Returns md5sum for a message object of type '<UD_NeckAngle-request>"
  "54d0093350d66ae9f81f65f132656855")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UD_NeckAngle-request)))
  "Returns md5sum for a message object of type 'UD_NeckAngle-request"
  "54d0093350d66ae9f81f65f132656855")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UD_NeckAngle-request>)))
  "Returns full string definition for message of type '<UD_NeckAngle-request>"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UD_NeckAngle-request)))
  "Returns full string definition for message of type 'UD_NeckAngle-request"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UD_NeckAngle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UD_NeckAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UD_NeckAngle-request
    (cl:cons ':finish (finish msg))
))
;//! \htmlinclude UD_NeckAngle-response.msg.html

(cl:defclass <UD_NeckAngle-response> (roslisp-msg-protocol:ros-message)
  ((ud_neckangle
    :reader ud_neckangle
    :initarg :ud_neckangle
    :type cl:float
    :initform 0.0))
)

(cl:defclass UD_NeckAngle-response (<UD_NeckAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UD_NeckAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UD_NeckAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<UD_NeckAngle-response> is deprecated: use ahra-srv:UD_NeckAngle-response instead.")))

(cl:ensure-generic-function 'ud_neckangle-val :lambda-list '(m))
(cl:defmethod ud_neckangle-val ((m <UD_NeckAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:ud_neckangle-val is deprecated.  Use ahra-srv:ud_neckangle instead.")
  (ud_neckangle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UD_NeckAngle-response>) ostream)
  "Serializes a message object of type '<UD_NeckAngle-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ud_neckangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UD_NeckAngle-response>) istream)
  "Deserializes a message object of type '<UD_NeckAngle-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ud_neckangle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UD_NeckAngle-response>)))
  "Returns string type for a service object of type '<UD_NeckAngle-response>"
  "ahra/UD_NeckAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UD_NeckAngle-response)))
  "Returns string type for a service object of type 'UD_NeckAngle-response"
  "ahra/UD_NeckAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UD_NeckAngle-response>)))
  "Returns md5sum for a message object of type '<UD_NeckAngle-response>"
  "54d0093350d66ae9f81f65f132656855")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UD_NeckAngle-response)))
  "Returns md5sum for a message object of type 'UD_NeckAngle-response"
  "54d0093350d66ae9f81f65f132656855")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UD_NeckAngle-response>)))
  "Returns full string definition for message of type '<UD_NeckAngle-response>"
  (cl:format cl:nil "float64 ud_neckangle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UD_NeckAngle-response)))
  "Returns full string definition for message of type 'UD_NeckAngle-response"
  (cl:format cl:nil "float64 ud_neckangle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UD_NeckAngle-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UD_NeckAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UD_NeckAngle-response
    (cl:cons ':ud_neckangle (ud_neckangle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UD_NeckAngle)))
  'UD_NeckAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UD_NeckAngle)))
  'UD_NeckAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UD_NeckAngle)))
  "Returns string type for a service object of type '<UD_NeckAngle>"
  "ahra/UD_NeckAngle")