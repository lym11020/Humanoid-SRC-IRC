; Auto-generated. Do not edit!


(cl:in-package ahra-srv)


;//! \htmlinclude SendMotion-request.msg.html

(cl:defclass <SendMotion-request> (roslisp-msg-protocol:ros-message)
  ((SM_finish
    :reader SM_finish
    :initarg :SM_finish
    :type cl:boolean
    :initform cl:nil)
   (TA_finish
    :reader TA_finish
    :initarg :TA_finish
    :type cl:boolean
    :initform cl:nil)
   (UD_finish
    :reader UD_finish
    :initarg :UD_finish
    :type cl:boolean
    :initform cl:nil)
   (RL_finish
    :reader RL_finish
    :initarg :RL_finish
    :type cl:boolean
    :initform cl:nil)
   (EM_finish
    :reader EM_finish
    :initarg :EM_finish
    :type cl:boolean
    :initform cl:nil)
   (ST_finish
    :reader ST_finish
    :initarg :ST_finish
    :type cl:boolean
    :initform cl:nil)
   (walkcount
    :reader walkcount
    :initarg :walkcount
    :type cl:integer
    :initform 0)
   (request_id
    :reader request_id
    :initarg :request_id
    :type cl:integer
    :initform 0))
)

(cl:defclass SendMotion-request (<SendMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<SendMotion-request> is deprecated: use ahra-srv:SendMotion-request instead.")))

(cl:ensure-generic-function 'SM_finish-val :lambda-list '(m))
(cl:defmethod SM_finish-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:SM_finish-val is deprecated.  Use ahra-srv:SM_finish instead.")
  (SM_finish m))

(cl:ensure-generic-function 'TA_finish-val :lambda-list '(m))
(cl:defmethod TA_finish-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:TA_finish-val is deprecated.  Use ahra-srv:TA_finish instead.")
  (TA_finish m))

(cl:ensure-generic-function 'UD_finish-val :lambda-list '(m))
(cl:defmethod UD_finish-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:UD_finish-val is deprecated.  Use ahra-srv:UD_finish instead.")
  (UD_finish m))

(cl:ensure-generic-function 'RL_finish-val :lambda-list '(m))
(cl:defmethod RL_finish-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:RL_finish-val is deprecated.  Use ahra-srv:RL_finish instead.")
  (RL_finish m))

(cl:ensure-generic-function 'EM_finish-val :lambda-list '(m))
(cl:defmethod EM_finish-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:EM_finish-val is deprecated.  Use ahra-srv:EM_finish instead.")
  (EM_finish m))

(cl:ensure-generic-function 'ST_finish-val :lambda-list '(m))
(cl:defmethod ST_finish-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:ST_finish-val is deprecated.  Use ahra-srv:ST_finish instead.")
  (ST_finish m))

(cl:ensure-generic-function 'walkcount-val :lambda-list '(m))
(cl:defmethod walkcount-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:walkcount-val is deprecated.  Use ahra-srv:walkcount instead.")
  (walkcount m))

(cl:ensure-generic-function 'request_id-val :lambda-list '(m))
(cl:defmethod request_id-val ((m <SendMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:request_id-val is deprecated.  Use ahra-srv:request_id instead.")
  (request_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendMotion-request>) ostream)
  "Serializes a message object of type '<SendMotion-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'SM_finish) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'TA_finish) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'UD_finish) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'RL_finish) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'EM_finish) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST_finish) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'walkcount)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'request_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendMotion-request>) istream)
  "Deserializes a message object of type '<SendMotion-request>"
    (cl:setf (cl:slot-value msg 'SM_finish) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'TA_finish) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'UD_finish) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'RL_finish) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'EM_finish) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST_finish) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'walkcount) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendMotion-request>)))
  "Returns string type for a service object of type '<SendMotion-request>"
  "ahra/SendMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendMotion-request)))
  "Returns string type for a service object of type 'SendMotion-request"
  "ahra/SendMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendMotion-request>)))
  "Returns md5sum for a message object of type '<SendMotion-request>"
  "195b77fcb35ce347682e382fd8b4d70b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendMotion-request)))
  "Returns md5sum for a message object of type 'SendMotion-request"
  "195b77fcb35ce347682e382fd8b4d70b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendMotion-request>)))
  "Returns full string definition for message of type '<SendMotion-request>"
  (cl:format cl:nil "bool SM_finish~%bool TA_finish~%bool UD_finish~%bool RL_finish~%bool EM_finish~%bool ST_finish~%int32 walkcount~%int32 request_id # Unique request ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendMotion-request)))
  "Returns full string definition for message of type 'SendMotion-request"
  (cl:format cl:nil "bool SM_finish~%bool TA_finish~%bool UD_finish~%bool RL_finish~%bool EM_finish~%bool ST_finish~%int32 walkcount~%int32 request_id # Unique request ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendMotion-request>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendMotion-request
    (cl:cons ':SM_finish (SM_finish msg))
    (cl:cons ':TA_finish (TA_finish msg))
    (cl:cons ':UD_finish (UD_finish msg))
    (cl:cons ':RL_finish (RL_finish msg))
    (cl:cons ':EM_finish (EM_finish msg))
    (cl:cons ':ST_finish (ST_finish msg))
    (cl:cons ':walkcount (walkcount msg))
    (cl:cons ':request_id (request_id msg))
))
;//! \htmlinclude SendMotion-response.msg.html

(cl:defclass <SendMotion-response> (roslisp-msg-protocol:ros-message)
  ((select_motion
    :reader select_motion
    :initarg :select_motion
    :type cl:fixnum
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (turn_angle
    :reader turn_angle
    :initarg :turn_angle
    :type cl:float
    :initform 0.0)
   (ud_neckangle
    :reader ud_neckangle
    :initarg :ud_neckangle
    :type cl:float
    :initform 0.0)
   (rl_neckangle
    :reader rl_neckangle
    :initarg :rl_neckangle
    :type cl:float
    :initform 0.0)
   (emergency
    :reader emergency
    :initarg :emergency
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SendMotion-response (<SendMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ahra-srv:<SendMotion-response> is deprecated: use ahra-srv:SendMotion-response instead.")))

(cl:ensure-generic-function 'select_motion-val :lambda-list '(m))
(cl:defmethod select_motion-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:select_motion-val is deprecated.  Use ahra-srv:select_motion instead.")
  (select_motion m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:distance-val is deprecated.  Use ahra-srv:distance instead.")
  (distance m))

(cl:ensure-generic-function 'turn_angle-val :lambda-list '(m))
(cl:defmethod turn_angle-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:turn_angle-val is deprecated.  Use ahra-srv:turn_angle instead.")
  (turn_angle m))

(cl:ensure-generic-function 'ud_neckangle-val :lambda-list '(m))
(cl:defmethod ud_neckangle-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:ud_neckangle-val is deprecated.  Use ahra-srv:ud_neckangle instead.")
  (ud_neckangle m))

(cl:ensure-generic-function 'rl_neckangle-val :lambda-list '(m))
(cl:defmethod rl_neckangle-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:rl_neckangle-val is deprecated.  Use ahra-srv:rl_neckangle instead.")
  (rl_neckangle m))

(cl:ensure-generic-function 'emergency-val :lambda-list '(m))
(cl:defmethod emergency-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:emergency-val is deprecated.  Use ahra-srv:emergency instead.")
  (emergency m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SendMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ahra-srv:success-val is deprecated.  Use ahra-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendMotion-response>) ostream)
  "Serializes a message object of type '<SendMotion-response>"
  (cl:let* ((signed (cl:slot-value msg 'select_motion)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'turn_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ud_neckangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rl_neckangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendMotion-response>) istream)
  "Deserializes a message object of type '<SendMotion-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'select_motion) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'turn_angle) (roslisp-utils:decode-double-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'emergency) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendMotion-response>)))
  "Returns string type for a service object of type '<SendMotion-response>"
  "ahra/SendMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendMotion-response)))
  "Returns string type for a service object of type 'SendMotion-response"
  "ahra/SendMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendMotion-response>)))
  "Returns md5sum for a message object of type '<SendMotion-response>"
  "195b77fcb35ce347682e382fd8b4d70b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendMotion-response)))
  "Returns md5sum for a message object of type 'SendMotion-response"
  "195b77fcb35ce347682e382fd8b4d70b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendMotion-response>)))
  "Returns full string definition for message of type '<SendMotion-response>"
  (cl:format cl:nil "int8 select_motion~%float64 distance~%float64 turn_angle~%float64 ud_neckangle~%float64 rl_neckangle~%bool emergency~%bool success # Response success status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendMotion-response)))
  "Returns full string definition for message of type 'SendMotion-response"
  (cl:format cl:nil "int8 select_motion~%float64 distance~%float64 turn_angle~%float64 ud_neckangle~%float64 rl_neckangle~%bool emergency~%bool success # Response success status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendMotion-response>))
  (cl:+ 0
     1
     8
     8
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendMotion-response
    (cl:cons ':select_motion (select_motion msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':turn_angle (turn_angle msg))
    (cl:cons ':ud_neckangle (ud_neckangle msg))
    (cl:cons ':rl_neckangle (rl_neckangle msg))
    (cl:cons ':emergency (emergency msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendMotion)))
  'SendMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendMotion)))
  'SendMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendMotion)))
  "Returns string type for a service object of type '<SendMotion>"
  "ahra/SendMotion")