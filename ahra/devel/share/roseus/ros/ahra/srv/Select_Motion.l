;; Auto-generated. Do not edit!


(when (boundp 'ahra::Select_Motion)
  (if (not (find-package "AHRA"))
    (make-package "AHRA"))
  (shadow 'Select_Motion (find-package "AHRA")))
(unless (find-package "AHRA::SELECT_MOTION")
  (make-package "AHRA::SELECT_MOTION"))
(unless (find-package "AHRA::SELECT_MOTIONREQUEST")
  (make-package "AHRA::SELECT_MOTIONREQUEST"))
(unless (find-package "AHRA::SELECT_MOTIONRESPONSE")
  (make-package "AHRA::SELECT_MOTIONRESPONSE"))

(in-package "ROS")





(defclass ahra::Select_MotionRequest
  :super ros::object
  :slots (_finish ))

(defmethod ahra::Select_MotionRequest
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

(defclass ahra::Select_MotionResponse
  :super ros::object
  :slots (_select_motion _distance ))

(defmethod ahra::Select_MotionResponse
  (:init
   (&key
    ((:select_motion __select_motion) 0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _select_motion (round __select_motion))
   (setq _distance (float __distance))
   self)
  (:select_motion
   (&optional __select_motion)
   (if __select_motion (setq _select_motion __select_motion)) _select_motion)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; int8 _select_motion
    1
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _select_motion
       (write-byte _select_motion s)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _select_motion
     (setq _select_motion (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _select_motion 127) (setq _select_motion (- _select_motion 256)))
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ahra::Select_Motion
  :super ros::object
  :slots ())

(setf (get ahra::Select_Motion :md5sum-) "d51f1f90cb25facd79c884f5f7a758e6")
(setf (get ahra::Select_Motion :datatype-) "ahra/Select_Motion")
(setf (get ahra::Select_Motion :request) ahra::Select_MotionRequest)
(setf (get ahra::Select_Motion :response) ahra::Select_MotionResponse)

(defmethod ahra::Select_MotionRequest
  (:response () (instance ahra::Select_MotionResponse :init)))

(setf (get ahra::Select_MotionRequest :md5sum-) "d51f1f90cb25facd79c884f5f7a758e6")
(setf (get ahra::Select_MotionRequest :datatype-) "ahra/Select_MotionRequest")
(setf (get ahra::Select_MotionRequest :definition-)
      "bool finish
---
int8 select_motion
float64 distance
")

(setf (get ahra::Select_MotionResponse :md5sum-) "d51f1f90cb25facd79c884f5f7a758e6")
(setf (get ahra::Select_MotionResponse :datatype-) "ahra/Select_MotionResponse")
(setf (get ahra::Select_MotionResponse :definition-)
      "bool finish
---
int8 select_motion
float64 distance
")



(provide :ahra/Select_Motion "d51f1f90cb25facd79c884f5f7a758e6")


