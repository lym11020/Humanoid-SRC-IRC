; Auto-generated. Do not edit!


(cl:in-package ahra-srv)


;//! \htmlinclude Emergency-request.msg.html

(cl:defclass <Emergency-request> (roslisp-msg-protocol:ros-message)
  ((finish
    :reader finish
    :initarg :finish
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Emergency-request (<Emergency-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Emergency-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Emergency-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<Emergency-request> is deprecated: use ahra-srv:Emergency-request instead.")))

(cl:ensure-generic-function 'finish-val :lambda-list '(m))
(cl:defmethod finish-val ((m <Emergency-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:finish-val is deprecated.  Use ahra-srv:finish instead.")
  (finish m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Emergency-request>) ostream)
  "Serializes a message object of type '<Emergency-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finish) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Emergency-request>) istream)
  "Deserializes a message object of type '<Emergency-request>"
    (cl:setf (cl:slot-value msg 'finish) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Emergency-request>)))
  "Returns string type for a service object of type '<Emergency-request>"
  "ahra/EmergencyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emergency-request)))
  "Returns string type for a service object of type 'Emergency-request"
  "ahra/EmergencyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Emergency-request>)))
  "Returns md5sum for a message object of type '<Emergency-request>"
  "04c66b9168769803386104918010c5ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Emergency-request)))
  "Returns md5sum for a message object of type 'Emergency-request"
  "04c66b9168769803386104918010c5ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Emergency-request>)))
  "Returns full string definition for message of type '<Emergency-request>"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Emergency-request)))
  "Returns full string definition for message of type 'Emergency-request"
  (cl:format cl:nil "bool finish~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Emergency-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Emergency-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Emergency-request
    (cl:cons ':finish (finish msg))
))
;//! \htmlinclude Emergency-response.msg.html

(cl:defclass <Emergency-response> (roslisp-msg-protocol:ros-message)
  ((emergency
    :reader emergency
    :initarg :emergency
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Emergency-response (<Emergency-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Emergency-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Emergency-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<Emergency-response> is deprecated: use ahra-srv:Emergency-response instead.")))

(cl:ensure-generic-function 'emergency-val :lambda-list '(m))
(cl:defmethod emergency-val ((m <Emergency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:emergency-val is deprecated.  Use ahra-srv:emergency instead.")
  (emergency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Emergency-response>) ostream)
  "Serializes a message object of type '<Emergency-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Emergency-response>) istream)
  "Deserializes a message object of type '<Emergency-response>"
    (cl:setf (cl:slot-value msg 'emergency) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Emergency-response>)))
  "Returns string type for a service object of type '<Emergency-response>"
  "ahra/EmergencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emergency-response)))
  "Returns string type for a service object of type 'Emergency-response"
  "ahra/EmergencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Emergency-response>)))
  "Returns md5sum for a message object of type '<Emergency-response>"
  "04c66b9168769803386104918010c5ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Emergency-response)))
  "Returns md5sum for a message object of type 'Emergency-response"
  "04c66b9168769803386104918010c5ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Emergency-response>)))
  "Returns full string definition for message of type '<Emergency-response>"
  (cl:format cl:nil "bool emergency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Emergency-response)))
  "Returns full string definition for message of type 'Emergency-response"
  (cl:format cl:nil "bool emergency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Emergency-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Emergency-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Emergency-response
    (cl:cons ':emergency (emergency msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Emergency)))
  'Emergency-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Emergency)))
  'Emergency-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Emergency)))
  "Returns string type for a service object of type '<Emergency>"
  "ahra/Emergency")