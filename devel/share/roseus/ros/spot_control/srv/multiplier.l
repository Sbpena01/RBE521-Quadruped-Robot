;; Auto-generated. Do not edit!


(when (boundp 'spot_control::multiplier)
  (if (not (find-package "SPOT_CONTROL"))
    (make-package "SPOT_CONTROL"))
  (shadow 'multiplier (find-package "SPOT_CONTROL")))
(unless (find-package "SPOT_CONTROL::MULTIPLIER")
  (make-package "SPOT_CONTROL::MULTIPLIER"))
(unless (find-package "SPOT_CONTROL::MULTIPLIERREQUEST")
  (make-package "SPOT_CONTROL::MULTIPLIERREQUEST"))
(unless (find-package "SPOT_CONTROL::MULTIPLIERRESPONSE")
  (make-package "SPOT_CONTROL::MULTIPLIERRESPONSE"))

(in-package "ROS")





(defclass spot_control::multiplierRequest
  :super ros::object
  :slots (_a _b ))

(defmethod spot_control::multiplierRequest
  (:init
   (&key
    ((:a __a) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   (setq _b (round __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; int32 _a
    4
    ;; int32 _b
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _a
       (write-long _a s)
     ;; int32 _b
       (write-long _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _a
     (setq _a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _b
     (setq _b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass spot_control::multiplierResponse
  :super ros::object
  :slots (_result ))

(defmethod spot_control::multiplierResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass spot_control::multiplier
  :super ros::object
  :slots ())

(setf (get spot_control::multiplier :md5sum-) "a7d7d7065d45755acef7d4dcf908162a")
(setf (get spot_control::multiplier :datatype-) "spot_control/multiplier")
(setf (get spot_control::multiplier :request) spot_control::multiplierRequest)
(setf (get spot_control::multiplier :response) spot_control::multiplierResponse)

(defmethod spot_control::multiplierRequest
  (:response () (instance spot_control::multiplierResponse :init)))

(setf (get spot_control::multiplierRequest :md5sum-) "a7d7d7065d45755acef7d4dcf908162a")
(setf (get spot_control::multiplierRequest :datatype-) "spot_control/multiplierRequest")
(setf (get spot_control::multiplierRequest :definition-)
      "int32 a
int32 b
---
int32 result
")

(setf (get spot_control::multiplierResponse :md5sum-) "a7d7d7065d45755acef7d4dcf908162a")
(setf (get spot_control::multiplierResponse :datatype-) "spot_control/multiplierResponse")
(setf (get spot_control::multiplierResponse :definition-)
      "int32 a
int32 b
---
int32 result
")



(provide :spot_control/multiplier "a7d7d7065d45755acef7d4dcf908162a")


