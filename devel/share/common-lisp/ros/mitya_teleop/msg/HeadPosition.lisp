; Auto-generated. Do not edit!


(cl:in-package mitya_teleop-msg)


;//! \htmlinclude HeadPosition.msg.html

(cl:defclass <HeadPosition> (roslisp-msg-protocol:ros-message)
  ((horizontal
    :reader horizontal
    :initarg :horizontal
    :type cl:float
    :initform 0.0)
   (vertical
    :reader vertical
    :initarg :vertical
    :type cl:float
    :initform 0.0))
)

(cl:defclass HeadPosition (<HeadPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HeadPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HeadPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mitya_teleop-msg:<HeadPosition> is deprecated: use mitya_teleop-msg:HeadPosition instead.")))

(cl:ensure-generic-function 'horizontal-val :lambda-list '(m))
(cl:defmethod horizontal-val ((m <HeadPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mitya_teleop-msg:horizontal-val is deprecated.  Use mitya_teleop-msg:horizontal instead.")
  (horizontal m))

(cl:ensure-generic-function 'vertical-val :lambda-list '(m))
(cl:defmethod vertical-val ((m <HeadPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mitya_teleop-msg:vertical-val is deprecated.  Use mitya_teleop-msg:vertical instead.")
  (vertical m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HeadPosition>) ostream)
  "Serializes a message object of type '<HeadPosition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'horizontal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vertical))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HeadPosition>) istream)
  "Deserializes a message object of type '<HeadPosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HeadPosition>)))
  "Returns string type for a message object of type '<HeadPosition>"
  "mitya_teleop/HeadPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HeadPosition)))
  "Returns string type for a message object of type 'HeadPosition"
  "mitya_teleop/HeadPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HeadPosition>)))
  "Returns md5sum for a message object of type '<HeadPosition>"
  "93817796a8d7b54c196b10312493371b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HeadPosition)))
  "Returns md5sum for a message object of type 'HeadPosition"
  "93817796a8d7b54c196b10312493371b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HeadPosition>)))
  "Returns full string definition for message of type '<HeadPosition>"
  (cl:format cl:nil "float32 horizontal~%float32 vertical~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HeadPosition)))
  "Returns full string definition for message of type 'HeadPosition"
  (cl:format cl:nil "float32 horizontal~%float32 vertical~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HeadPosition>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HeadPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'HeadPosition
    (cl:cons ':horizontal (horizontal msg))
    (cl:cons ':vertical (vertical msg))
))
