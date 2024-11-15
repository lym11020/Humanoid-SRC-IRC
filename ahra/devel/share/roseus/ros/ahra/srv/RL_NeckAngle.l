;; Auto-generated. Do not edit!


(when (boundp 'ahra::RL_NeckAngle)
  (if (not (find-package "AHRA"))
    (make-package "AHRA"))
  (shadow 'RL_NeckAngle (find-package "AHRA")))
(unless (find-package "AHRA::RL_NECKANGLE")
  (make-package "AHRA::RL_NECKANGLE"))
(unless (find-package "AHRA::RL_NECKANGLEREQUEST")
  (make-package "AHRA::RL_NECKANGLEREQUEST"))
(unless (find-package "AHRA::RL_NECKANGLERESPONSE")
  (make-package "AHRA::RL_NECKANGLERESPONSE"))

(in-package "ROS")





(defclass ahra::RL_NeckAngleRequest
  :super ros::object
  :slots (_finish ))

(defmethod ahra::RL_NeckAngleRequest
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

(defclass ahra::RL_NeckAngleResponse
  :super ros::object
  :slots (_rl_neckangle ))

(defmethod ahra::RL_NeckAngleResponse
  (:init
   (&key
    ((:rl_neckangle __rl_neckangle) 0.0)
    )
   (send-super :init)
   (setq _rl_neckangle (float __rl_neckangle))
   self)
  (:rl_neckangle
   (&optional __rl_neckangle)
   (if __rl_neckangle (setq _rl_neckangle __rl_neckangle)) _rl_neckangle)
  (:serialization-length
   ()
   (+
    ;; float64 _rl_neckangle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _rl_neckangle
       (sys::poke _rl_neckangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _rl_neckangle
     (setq _rl_neckangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ahra::RL_NeckAngle
  :super ros::object
  :slots ())

(setf (get ahra::RL_NeckAngle :md5sum-) "53641ec2b7d57c4826404748be4e88f1")
(setf (get ahra::RL_NeckAngle :datatype-) "ahra/RL_NeckAngle")
(setf (get ahra::RL_NeckAngle :request) ahra::RL_NeckAngleRequest)
(setf (get ahra::RL_NeckAngle :response) ahra::RL_NeckAngleResponse)

(defmethod ahra::RL_NeckAngleRequest
  (:response () (instance ahra::RL_NeckAngleResponse :init)))

(setf (get ahra::RL_NeckAngleRequest :md5sum-) "53641ec2b7d57c4826404748be4e88f1")
(setf (get ahra::RL_NeckAngleRequest :datatype-) "ahra/RL_NeckAngleRequest")
(setf (get ahra::RL_NeckAngleRequest :definition-)
      "bool finish
---
float64 rl_neckangle
")

(setf (get ahra::RL_NeckAngleResponse :md5sum-) "53641ec2b7d57c4826404748be4e88f1")
(setf (get ahra::RL_NeckAngleResponse :datatype-) "ahra/RL_NeckAngleResponse")
(setf (get ahra::RL_NeckAngleResponse :definition-)
      "bool finish
---
float64 rl_neckangle
")



(provide :ahra/RL_NeckAngle "53641ec2b7d57c4826404748be4e88f1")


