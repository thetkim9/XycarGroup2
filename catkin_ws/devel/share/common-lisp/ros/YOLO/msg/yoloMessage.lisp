; Auto-generated. Do not edit!


(cl:in-package YOLO-msg)


;//! \htmlinclude yoloMessage.msg.html

(cl:defclass <yoloMessage> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass yoloMessage (<yoloMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <yoloMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'yoloMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name YOLO-msg:<yoloMessage> is deprecated: use YOLO-msg:yoloMessage instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <yoloMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader YOLO-msg:data-val is deprecated.  Use YOLO-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <yoloMessage>) ostream)
  "Serializes a message object of type '<yoloMessage>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <yoloMessage>) istream)
  "Deserializes a message object of type '<yoloMessage>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<yoloMessage>)))
  "Returns string type for a message object of type '<yoloMessage>"
  "YOLO/yoloMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'yoloMessage)))
  "Returns string type for a message object of type 'yoloMessage"
  "YOLO/yoloMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<yoloMessage>)))
  "Returns md5sum for a message object of type '<yoloMessage>"
  "cce5a364f3a3be12c9722c6dcad2fa94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'yoloMessage)))
  "Returns md5sum for a message object of type 'yoloMessage"
  "cce5a364f3a3be12c9722c6dcad2fa94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<yoloMessage>)))
  "Returns full string definition for message of type '<yoloMessage>"
  (cl:format cl:nil "string[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'yoloMessage)))
  "Returns full string definition for message of type 'yoloMessage"
  (cl:format cl:nil "string[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <yoloMessage>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <yoloMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'yoloMessage
    (cl:cons ':data (data msg))
))
