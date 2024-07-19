;; Auto-generated. Do not edit!


(when (boundp 'spot_control::jointControl)
  (if (not (find-package "SPOT_CONTROL"))
    (make-package "SPOT_CONTROL"))
  (shadow 'jointControl (find-package "SPOT_CONTROL")))
(unless (find-package "SPOT_CONTROL::JOINTCONTROL")
  (make-package "SPOT_CONTROL::JOINTCONTROL"))
(unless (find-package "SPOT_CONTROL::JOINTCONTROLREQUEST")
  (make-package "SPOT_CONTROL::JOINTCONTROLREQUEST"))
(unless (find-package "SPOT_CONTROL::JOINTCONTROLRESPONSE")
  (make-package "SPOT_CONTROL::JOINTCONTROLRESPONSE"))

(in-package "ROS")





(defclass spot_control::jointControlRequest
  :super ros::object
  :slots (_joint_angles ))

(defmethod spot_control::jointControlRequest
  (:init
   (&key
    ((:joint_angles __joint_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   self)
  (:joint_angles
   (&optional __joint_angles)
   (if __joint_angles (setq _joint_angles __joint_angles)) _joint_angles)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_angles
    (* 8    (length _joint_angles)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_angles
     (write-long (length _joint_angles) s)
     (dotimes (i (length _joint_angles))
       (sys::poke (elt _joint_angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass spot_control::jointControlResponse
  :super ros::object
  :slots (_success _message ))

(defmethod spot_control::jointControlResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass spot_control::jointControl
  :super ros::object
  :slots ())

(setf (get spot_control::jointControl :md5sum-) "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(setf (get spot_control::jointControl :datatype-) "spot_control/jointControl")
(setf (get spot_control::jointControl :request) spot_control::jointControlRequest)
(setf (get spot_control::jointControl :response) spot_control::jointControlResponse)

(defmethod spot_control::jointControlRequest
  (:response () (instance spot_control::jointControlResponse :init)))

(setf (get spot_control::jointControlRequest :md5sum-) "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(setf (get spot_control::jointControlRequest :datatype-) "spot_control/jointControlRequest")
(setf (get spot_control::jointControlRequest :definition-)
      "# Request
float64[] joint_angles  # A list of target angles for the joints of all legs
---
# Response
bool success  # Whether the joint commands were successfully executed
string message
")

(setf (get spot_control::jointControlResponse :md5sum-) "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(setf (get spot_control::jointControlResponse :datatype-) "spot_control/jointControlResponse")
(setf (get spot_control::jointControlResponse :definition-)
      "# Request
float64[] joint_angles  # A list of target angles for the joints of all legs
---
# Response
bool success  # Whether the joint commands were successfully executed
string message
")



(provide :spot_control/jointControl "ba9d5e6dc2edb0be1ff0f44aa41279a2")


