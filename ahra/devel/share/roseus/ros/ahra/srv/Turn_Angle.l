;; Auto-generated. Do not edit!


(when (boundp 'ahra::Turn_Angle)
  (if (not (find-package "AHRA"))
    (make-package "AHRA"))
  (shadow 'Turn_Angle (find-package "AHRA")))
(unless (find-package "AHRA::TURN_ANGLE")
  (make-package "AHRA::TURN_ANGLE"))
(unless (find-package "AHRA::TURN_ANGLEREQUEST")
  (make-package "AHRA::TURN_ANGLEREQUEST"))
(unless (find-package "AHRA::TURN_ANGLERESPONSE")
  (make-package "AHRA::TURN_ANGLERESPONSE"))

(in-package "ROS")





(defclass ahra::Turn_AngleRequest
  :super ros::object
  :slots (_finish ))

(defmethod ahra::Turn_AngleRequest
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

(defclass ahra::Turn_AngleResponse
  :super ros::object
  :slots (_turn_angle ))

(defmethod ahra::Turn_AngleResponse
  (:init
   (&key
    ((:turn_angle __turn_angle) 0.0)
    )
   (send-super :init)
   (setq _turn_angle (float __turn_angle))
   self)
  (:turn_angle
   (&optional __turn_angle)
   (if __turn_angle (setq _turn_angle __turn_angle)) _turn_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _turn_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _turn_angle
       (sys::poke _turn_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _turn_angle
     (setq _turn_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ahra::Turn_Angle
  :super ros::object
  :slots ())

(setf (get ahra::Turn_Angle :md5sum-) "2d06fbbfad695cc7cf264315ed9bddd9")
(setf (get ahra::Turn_Angle :datatype-) "ahra/Turn_Angle")
(setf (get ahra::Turn_Angle :request) ahra::Turn_AngleRequest)
(setf (get ahra::Turn_Angle :response) ahra::Turn_AngleResponse)

(defmethod ahra::Turn_AngleRequest
  (:response () (instance ahra::Turn_AngleResponse :init)))

(setf (get ahra::Turn_AngleRequest :md5sum-) "2d06fbbfad695cc7cf264315ed9bddd9")
(setf (get ahra::Turn_AngleRequest :datatype-) "ahra/Turn_AngleRequest")
(setf (get ahra::Turn_AngleRequest :definition-)
      "bool finish
---
float64 turn_angle
")

(setf (get ahra::Turn_AngleResponse :md5sum-) "2d06fbbfad695cc7cf264315ed9bddd9")
(setf (get ahra::Turn_AngleResponse :datatype-) "ahra/Turn_AngleResponse")
(setf (get ahra::Turn_AngleResponse :definition-)
      "bool finish
---
float64 turn_angle
")



(provide :ahra/Turn_Angle "2d06fbbfad695cc7cf264315ed9bddd9")


