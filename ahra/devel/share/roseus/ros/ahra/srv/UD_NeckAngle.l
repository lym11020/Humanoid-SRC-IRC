;; Auto-generated. Do not edit!


(when (boundp 'ahra::UD_NeckAngle)
  (if (not (find-package "AHRA"))
    (make-package "AHRA"))
  (shadow 'UD_NeckAngle (find-package "AHRA")))
(unless (find-package "AHRA::UD_NECKANGLE")
  (make-package "AHRA::UD_NECKANGLE"))
(unless (find-package "AHRA::UD_NECKANGLEREQUEST")
  (make-package "AHRA::UD_NECKANGLEREQUEST"))
(unless (find-package "AHRA::UD_NECKANGLERESPONSE")
  (make-package "AHRA::UD_NECKANGLERESPONSE"))

(in-package "ROS")





(defclass ahra::UD_NeckAngleRequest
  :super ros::object
  :slots (_finish ))

(defmethod ahra::UD_NeckAngleRequest
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

(defclass ahra::UD_NeckAngleResponse
  :super ros::object
  :slots (_ud_neckangle ))

(defmethod ahra::UD_NeckAngleResponse
  (:init
   (&key
    ((:ud_neckangle __ud_neckangle) 0.0)
    )
   (send-super :init)
   (setq _ud_neckangle (float __ud_neckangle))
   self)
  (:ud_neckangle
   (&optional __ud_neckangle)
   (if __ud_neckangle (setq _ud_neckangle __ud_neckangle)) _ud_neckangle)
  (:serialization-length
   ()
   (+
    ;; float64 _ud_neckangle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _ud_neckangle
       (sys::poke _ud_neckangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _ud_neckangle
     (setq _ud_neckangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ahra::UD_NeckAngle
  :super ros::object
  :slots ())

(setf (get ahra::UD_NeckAngle :md5sum-) "54d0093350d66ae9f81f65f132656855")
(setf (get ahra::UD_NeckAngle :datatype-) "ahra/UD_NeckAngle")
(setf (get ahra::UD_NeckAngle :request) ahra::UD_NeckAngleRequest)
(setf (get ahra::UD_NeckAngle :response) ahra::UD_NeckAngleResponse)

(defmethod ahra::UD_NeckAngleRequest
  (:response () (instance ahra::UD_NeckAngleResponse :init)))

(setf (get ahra::UD_NeckAngleRequest :md5sum-) "54d0093350d66ae9f81f65f132656855")
(setf (get ahra::UD_NeckAngleRequest :datatype-) "ahra/UD_NeckAngleRequest")
(setf (get ahra::UD_NeckAngleRequest :definition-)
      "bool finish
---
float64 ud_neckangle
")

(setf (get ahra::UD_NeckAngleResponse :md5sum-) "54d0093350d66ae9f81f65f132656855")
(setf (get ahra::UD_NeckAngleResponse :datatype-) "ahra/UD_NeckAngleResponse")
(setf (get ahra::UD_NeckAngleResponse :definition-)
      "bool finish
---
float64 ud_neckangle
")



(provide :ahra/UD_NeckAngle "54d0093350d66ae9f81f65f132656855")


