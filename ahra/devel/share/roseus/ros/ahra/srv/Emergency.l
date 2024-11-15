;; Auto-generated. Do not edit!


(when (boundp 'ahra::Emergency)
  (if (not (find-package "AHRA"))
    (make-package "AHRA"))
  (shadow 'Emergency (find-package "AHRA")))
(unless (find-package "AHRA::EMERGENCY")
  (make-package "AHRA::EMERGENCY"))
(unless (find-package "AHRA::EMERGENCYREQUEST")
  (make-package "AHRA::EMERGENCYREQUEST"))
(unless (find-package "AHRA::EMERGENCYRESPONSE")
  (make-package "AHRA::EMERGENCYRESPONSE"))

(in-package "ROS")





(defclass ahra::EmergencyRequest
  :super ros::object
  :slots (_finish ))

(defmethod ahra::EmergencyRequest
  (:init
   (&key
    ((:finish __finish) nil)
    )
   (send-super :init)
   (setq _finish __finish)
   self)
  (:finish
   (&optional (__finish :null))
   (if (not (eq __finish :null)) (setq _finish __finish)) _finish)
  (:serialization-length
   ()
   (+
    ;; bool _finish
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _finish
       (if _finish (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _finish
     (setq _finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ahra::EmergencyResponse
  :super ros::object
  :slots (_emergency ))

(defmethod ahra::EmergencyResponse
  (:init
   (&key
    ((:emergency __emergency) nil)
    )
   (send-super :init)
   (setq _emergency __emergency)
   self)
  (:emergency
   (&optional (__emergency :null))
   (if (not (eq __emergency :null)) (setq _emergency __emergency)) _emergency)
  (:serialization-length
   ()
   (+
    ;; bool _emergency
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _emergency
       (if _emergency (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _emergency
     (setq _emergency (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ahra::Emergency
  :super ros::object
  :slots ())

(setf (get ahra::Emergency :md5sum-) "04c66b9168769803386104918010c5ae")
(setf (get ahra::Emergency :datatype-) "ahra/Emergency")
(setf (get ahra::Emergency :request) ahra::EmergencyRequest)
(setf (get ahra::Emergency :response) ahra::EmergencyResponse)

(defmethod ahra::EmergencyRequest
  (:response () (instance ahra::EmergencyResponse :init)))

(setf (get ahra::EmergencyRequest :md5sum-) "04c66b9168769803386104918010c5ae")
(setf (get ahra::EmergencyRequest :datatype-) "ahra/EmergencyRequest")
(setf (get ahra::EmergencyRequest :definition-)
      "bool finish
---
bool emergency
")

(setf (get ahra::EmergencyResponse :md5sum-) "04c66b9168769803386104918010c5ae")
(setf (get ahra::EmergencyResponse :datatype-) "ahra/EmergencyResponse")
(setf (get ahra::EmergencyResponse :definition-)
      "bool finish
---
bool emergency
")



(provide :ahra/Emergency "04c66b9168769803386104918010c5ae")


