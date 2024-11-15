; Auto-generated. Do not edit!


(cl:in-package ahra-srv)


;//! \htmlinclude RL_NeckAngle-request.msg.html

(cl:defclass <RL_NeckAngle-request> (roslisp-msg-protocol:ros-message)
  ((finish
    :reader finish
    :initarg :finish
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RL_NeckAngle-request (<RL_NeckAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RL_NeckAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RL_NeckAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<RL_NeckAngle-request> is deprecated: use ahra-srv:RL_NeckAngle-request instead.")))

(cl:ensure-generic-function 'finish-val :lambda-list '(m))
(cl:defmethod finish-val ((m <RL_NeckAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:finish-val is deprecated.  Use ahra-srv:finish instead.")
  (finish m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RL_NeckAngle-request>) ostream)
  "Serializes a message object of type '<RL_NeckAngle-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finish) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RL_NeckAngle-request>) istream)
  "Deserializes a message object of type '<RL_NeckAngle-request>"
    (cl:setf (cl:slot-value msg 'finish) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RL_NeckAngle-request>)))
  "Returns string type for a service object of type '<RL_NeckAngle-request>"
  "ahra/RL_NeckAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RL_NeckAngle-request)))
  "Returns string type for a service object of type 'RL_NeckAngle-request"
  "ahra/RL_NeckAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RL_NeckAngle-request>)))
  "Returns md5sum for a message object of type '<RL_NeckAngle-request>"
  "53641ec2b7d57c4826404748be4e88f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RL_NeckAngle-request)))
  "Returns md5sum for a message object of type 'RL_NeckAngle-request"
  "53641ec2b7d57c4826404748be4e88f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RL_NeckAngle-request>)))
  "Returns full string definition for message of type '<RL_NeckAngle-request>"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RL_NeckAngle-request)))
  "Returns full string definition for message of type 'RL_NeckAngle-request"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RL_NeckAngle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RL_NeckAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RL_NeckAngle-request
    (cl:cons ':finish (finish msg))
))
;//! \htmlinclude RL_NeckAngle-response.msg.html

(cl:defclass <RL_NeckAngle-response> (roslisp-msg-protocol:ros-message)
  ((rl_neckangle
    :reader rl_neckangle
    :initarg :rl_neckangle
    :type cl:float
    :initform 0.0))
)

(cl:defclass RL_NeckAngle-response (<RL_NeckAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RL_NeckAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RL_NeckAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<RL_NeckAngle-response> is deprecated: use ahra-srv:RL_NeckAngle-response instead.")))

(cl:ensure-generic-function 'rl_neckangle-val :lambda-list '(m))
(cl:defmethod rl_neckangle-val ((m <RL_NeckAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:rl_neckangle-val is deprecated.  Use ahra-srv:rl_neckangle instead.")
  (rl_neckangle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RL_NeckAngle-response>) ostream)
  "Serializes a message object of type '<RL_NeckAngle-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rl_neckangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RL_NeckAngle-response>) istream)
  "Deserializes a message object of type '<RL_NeckAngle-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rl_neckangle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RL_NeckAngle-response>)))
  "Returns string type for a service object of type '<RL_NeckAngle-response>"
  "ahra/RL_NeckAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RL_NeckAngle-response)))
  "Returns string type for a service object of type 'RL_NeckAngle-response"
  "ahra/RL_NeckAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RL_NeckAngle-response>)))
  "Returns md5sum for a message object of type '<RL_NeckAngle-response>"
  "53641ec2b7d57c4826404748be4e88f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RL_NeckAngle-response)))
  "Returns md5sum for a message object of type 'RL_NeckAngle-response"
  "53641ec2b7d57c4826404748be4e88f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RL_NeckAngle-response>)))
  "Returns full string definition for message of type '<RL_NeckAngle-response>"
  (cl:format cl:nil "float64 rl_neckangle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RL_NeckAngle-response)))
  "Returns full string definition for message of type 'RL_NeckAngle-response"
  (cl:format cl:nil "float64 rl_neckangle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RL_NeckAngle-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RL_NeckAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RL_NeckAngle-response
    (cl:cons ':rl_neckangle (rl_neckangle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RL_NeckAngle)))
  'RL_NeckAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RL_NeckAngle)))
  'RL_NeckAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RL_NeckAngle)))
  "Returns string type for a service object of type '<RL_NeckAngle>"
  "ahra/RL_NeckAngle")