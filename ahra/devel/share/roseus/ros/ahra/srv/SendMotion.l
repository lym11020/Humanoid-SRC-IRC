;; Auto-generated. Do not edit!


(when (boundp 'ahra::SendMotion)
  (if (not (find-package "AHRA"))
    (make-package "AHRA"))
  (shadow 'SendMotion (find-package "AHRA")))
(unless (find-package "AHRA::SENDMOTION")
  (make-package "AHRA::SENDMOTION"))
(unless (find-package "AHRA::SENDMOTIONREQUEST")
  (make-package "AHRA::SENDMOTIONREQUEST"))
(unless (find-package "AHRA::SENDMOTIONRESPONSE")
  (make-package "AHRA::SENDMOTIONRESPONSE"))

(in-package "ROS")





(defclass ahra::SendMotionRequest
  :super ros::object
  :slots (_SM_finish _TA_finish _UD_finish _RL_finish _EM_finish _ST_finish _walkcount _request_id ))

(defmethod ahra::SendMotionRequest
  (:init
   (&key
    ((:SM_finish __SM_finish) nil)
    ((:TA_finish __TA_finish) nil)
    ((:UD_finish __UD_finish) nil)
    ((:RL_finish __RL_finish) nil)
    ((:EM_finish __EM_finish) nil)
    ((:ST_finish __ST_finish) nil)
    ((:walkcount __walkcount) 0)
    ((:request_id __request_id) 0)
    )
   (send-super :init)
   (setq _SM_finish __SM_finish)
   (setq _TA_finish __TA_finish)
   (setq _UD_finish __UD_finish)
   (setq _RL_finish __RL_finish)
   (setq _EM_finish __EM_finish)
   (setq _ST_finish __ST_finish)
   (setq _walkcount (round __walkcount))
   (setq _request_id (round __request_id))
   self)
  (:SM_finish
   (&optional (__SM_finish :null))
   (if (not (eq __SM_finish :null)) (setq _SM_finish __SM_finish)) _SM_finish)
  (:TA_finish
   (&optional (__TA_finish :null))
   (if (not (eq __TA_finish :null)) (setq _TA_finish __TA_finish)) _TA_finish)
  (:UD_finish
   (&optional (__UD_finish :null))
   (if (not (eq __UD_finish :null)) (setq _UD_finish __UD_finish)) _UD_finish)
  (:RL_finish
   (&optional (__RL_finish :null))
   (if (not (eq __RL_finish :null)) (setq _RL_finish __RL_finish)) _RL_finish)
  (:EM_finish
   (&optional (__EM_finish :null))
   (if (not (eq __EM_finish :null)) (setq _EM_finish __EM_finish)) _EM_finish)
  (:ST_finish
   (&optional (__ST_finish :null))
   (if (not (eq __ST_finish :null)) (setq _ST_finish __ST_finish)) _ST_finish)
  (:walkcount
   (&optional __walkcount)
   (if __walkcount (setq _walkcount __walkcount)) _walkcount)
  (:request_id
   (&optional __request_id)
   (if __request_id (setq _request_id __request_id)) _request_id)
  (:serialization-length
   ()
   (+
    ;; bool _SM_finish
    1
    ;; bool _TA_finish
    1
    ;; bool _UD_finish
    1
    ;; bool _RL_finish
    1
    ;; bool _EM_finish
    1
    ;; bool _ST_finish
    1
    ;; int32 _walkcount
    4
    ;; int32 _request_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _SM_finish
       (if _SM_finish (write-byte -1 s) (write-byte 0 s))
     ;; bool _TA_finish
       (if _TA_finish (write-byte -1 s) (write-byte 0 s))
     ;; bool _UD_finish
       (if _UD_finish (write-byte -1 s) (write-byte 0 s))
     ;; bool _RL_finish
       (if _RL_finish (write-byte -1 s) (write-byte 0 s))
     ;; bool _EM_finish
       (if _EM_finish (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST_finish
       (if _ST_finish (write-byte -1 s) (write-byte 0 s))
     ;; int32 _walkcount
       (write-long _walkcount s)
     ;; int32 _request_id
       (write-long _request_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _SM_finish
     (setq _SM_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _TA_finish
     (setq _TA_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _UD_finish
     (setq _UD_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _RL_finish
     (setq _RL_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _EM_finish
     (setq _EM_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST_finish
     (setq _ST_finish (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _walkcount
     (setq _walkcount (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _request_id
     (setq _request_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ahra::SendMotionResponse
  :super ros::object
  :slots (_select_motion _distance _turn_angle _ud_neckangle _rl_neckangle _emergency _success ))

(defmethod ahra::SendMotionResponse
  (:init
   (&key
    ((:select_motion __select_motion) 0)
    ((:distance __distance) 0.0)
    ((:turn_angle __turn_angle) 0.0)
    ((:ud_neckangle __ud_neckangle) 0.0)
    ((:rl_neckangle __rl_neckangle) 0.0)
    ((:emergency __emergency) nil)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _select_motion (round __select_motion))
   (setq _distance (float __distance))
   (setq _turn_angle (float __turn_angle))
   (setq _ud_neckangle (float __ud_neckangle))
   (setq _rl_neckangle (float __rl_neckangle))
   (setq _emergency __emergency)
   (setq _success __success)
   self)
  (:select_motion
   (&optional __select_motion)
   (if __select_motion (setq _select_motion __select_motion)) _select_motion)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:turn_angle
   (&optional __turn_angle)
   (if __turn_angle (setq _turn_angle __turn_angle)) _turn_angle)
  (:ud_neckangle
   (&optional __ud_neckangle)
   (if __ud_neckangle (setq _ud_neckangle __ud_neckangle)) _ud_neckangle)
  (:rl_neckangle
   (&optional __rl_neckangle)
   (if __rl_neckangle (setq _rl_neckangle __rl_neckangle)) _rl_neckangle)
  (:emergency
   (&optional (__emergency :null))
   (if (not (eq __emergency :null)) (setq _emergency __emergency)) _emergency)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _select_motion
    1
    ;; float64 _distance
    8
    ;; float64 _turn_angle
    8
    ;; float64 _ud_neckangle
    8
    ;; float64 _rl_neckangle
    8
    ;; bool _emergency
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _select_motion
       (write-byte _select_motion s)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _turn_angle
       (sys::poke _turn_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ud_neckangle
       (sys::poke _ud_neckangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rl_neckangle
       (sys::poke _rl_neckangle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _emergency
       (if _emergency (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _select_motion
     (setq _select_motion (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _select_motion 127) (setq _select_motion (- _select_motion 256)))
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _turn_angle
     (setq _turn_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ud_neckangle
     (setq _ud_neckangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rl_neckangle
     (setq _rl_neckangle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _emergency
     (setq _emergency (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ahra::SendMotion
  :super ros::object
  :slots ())

(setf (get ahra::SendMotion :md5sum-) "195b77fcb35ce347682e382fd8b4d70b")
(setf (get ahra::SendMotion :datatype-) "ahra/SendMotion")
(setf (get ahra::SendMotion :request) ahra::SendMotionRequest)
(setf (get ahra::SendMotion :response) ahra::SendMotionResponse)

(defmethod ahra::SendMotionRequest
  (:response () (instance ahra::SendMotionResponse :init)))

(setf (get ahra::SendMotionRequest :md5sum-) "195b77fcb35ce347682e382fd8b4d70b")
(setf (get ahra::SendMotionRequest :datatype-) "ahra/SendMotionRequest")
(setf (get ahra::SendMotionRequest :definition-)
      "bool SM_finish
bool TA_finish
bool UD_finish
bool RL_finish
bool EM_finish
bool ST_finish
int32 walkcount
int32 request_id # Unique request ID
---
int8 select_motion
float64 distance
float64 turn_angle
float64 ud_neckangle
float64 rl_neckangle
bool emergency
bool success # Response success status
")

(setf (get ahra::SendMotionResponse :md5sum-) "195b77fcb35ce347682e382fd8b4d70b")
(setf (get ahra::SendMotionResponse :datatype-) "ahra/SendMotionResponse")
(setf (get ahra::SendMotionResponse :definition-)
      "bool SM_finish
bool TA_finish
bool UD_finish
bool RL_finish
bool EM_finish
bool ST_finish
int32 walkcount
int32 request_id # Unique request ID
---
int8 select_motion
float64 distance
float64 turn_angle
float64 ud_neckangle
float64 rl_neckangle
bool emergency
bool success # Response success status
")



(provide :ahra/SendMotion "195b77fcb35ce347682e382fd8b4d70b")


