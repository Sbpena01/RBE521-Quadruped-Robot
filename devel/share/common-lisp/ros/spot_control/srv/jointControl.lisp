; Auto-generated. Do not edit!


(cl:in-package spot_control-srv)


;//! \htmlinclude jointControl-request.msg.html

(cl:defclass <jointControl-request> (roslisp-msg-protocol:ros-message)
  ((joint_angles
    :reader joint_angles
    :initarg :joint_angles
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass jointControl-request (<jointControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jointControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jointControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spot_control-srv:<jointControl-request> is deprecated: use spot_control-srv:jointControl-request instead.")))

(cl:ensure-generic-function 'joint_angles-val :lambda-list '(m))
(cl:defmethod joint_angles-val ((m <jointControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spot_control-srv:joint_angles-val is deprecated.  Use spot_control-srv:joint_angles instead.")
  (joint_angles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jointControl-request>) ostream)
  "Serializes a message object of type '<jointControl-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_angles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_angles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jointControl-request>) istream)
  "Deserializes a message object of type '<jointControl-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_angles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_angles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jointControl-request>)))
  "Returns string type for a service object of type '<jointControl-request>"
  "spot_control/jointControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jointControl-request)))
  "Returns string type for a service object of type 'jointControl-request"
  "spot_control/jointControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jointControl-request>)))
  "Returns md5sum for a message object of type '<jointControl-request>"
  "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jointControl-request)))
  "Returns md5sum for a message object of type 'jointControl-request"
  "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jointControl-request>)))
  "Returns full string definition for message of type '<jointControl-request>"
  (cl:format cl:nil "# Request~%float64[] joint_angles  # A list of target angles for the joints of all legs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jointControl-request)))
  "Returns full string definition for message of type 'jointControl-request"
  (cl:format cl:nil "# Request~%float64[] joint_angles  # A list of target angles for the joints of all legs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jointControl-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_angles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jointControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'jointControl-request
    (cl:cons ':joint_angles (joint_angles msg))
))
;//! \htmlinclude jointControl-response.msg.html

(cl:defclass <jointControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass jointControl-response (<jointControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jointControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jointControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spot_control-srv:<jointControl-response> is deprecated: use spot_control-srv:jointControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <jointControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spot_control-srv:success-val is deprecated.  Use spot_control-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <jointControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spot_control-srv:message-val is deprecated.  Use spot_control-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jointControl-response>) ostream)
  "Serializes a message object of type '<jointControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jointControl-response>) istream)
  "Deserializes a message object of type '<jointControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jointControl-response>)))
  "Returns string type for a service object of type '<jointControl-response>"
  "spot_control/jointControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jointControl-response)))
  "Returns string type for a service object of type 'jointControl-response"
  "spot_control/jointControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jointControl-response>)))
  "Returns md5sum for a message object of type '<jointControl-response>"
  "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jointControl-response)))
  "Returns md5sum for a message object of type 'jointControl-response"
  "ba9d5e6dc2edb0be1ff0f44aa41279a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jointControl-response>)))
  "Returns full string definition for message of type '<jointControl-response>"
  (cl:format cl:nil "# Response~%bool success  # Whether the joint commands were successfully executed~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jointControl-response)))
  "Returns full string definition for message of type 'jointControl-response"
  (cl:format cl:nil "# Response~%bool success  # Whether the joint commands were successfully executed~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jointControl-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jointControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'jointControl-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'jointControl)))
  'jointControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'jointControl)))
  'jointControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jointControl)))
  "Returns string type for a service object of type '<jointControl>"
  "spot_control/jointControl")