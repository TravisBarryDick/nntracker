; Auto-generated. Do not edit!


(cl:in-package NNTracker-msg)


;//! \htmlinclude NNTrackerROI.msg.html

(cl:defclass <NNTrackerROI> (roslisp-msg-protocol:ros-message)
  ((ulx
    :reader ulx
    :initarg :ulx
    :type cl:float
    :initform 0.0)
   (uly
    :reader uly
    :initarg :uly
    :type cl:float
    :initform 0.0)
   (urx
    :reader urx
    :initarg :urx
    :type cl:float
    :initform 0.0)
   (ury
    :reader ury
    :initarg :ury
    :type cl:float
    :initform 0.0)
   (lrx
    :reader lrx
    :initarg :lrx
    :type cl:float
    :initform 0.0)
   (lry
    :reader lry
    :initarg :lry
    :type cl:float
    :initform 0.0)
   (llx
    :reader llx
    :initarg :llx
    :type cl:float
    :initform 0.0)
   (lly
    :reader lly
    :initarg :lly
    :type cl:float
    :initform 0.0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0)
   (perimeter
    :reader perimeter
    :initarg :perimeter
    :type cl:float
    :initform 0.0)
   (cmx
    :reader cmx
    :initarg :cmx
    :type cl:float
    :initform 0.0)
   (cmy
    :reader cmy
    :initarg :cmy
    :type cl:float
    :initform 0.0))
)

(cl:defclass NNTrackerROI (<NNTrackerROI>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NNTrackerROI>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NNTrackerROI)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name NNTracker-msg:<NNTrackerROI> is deprecated: use NNTracker-msg:NNTrackerROI instead.")))

(cl:ensure-generic-function 'ulx-val :lambda-list '(m))
(cl:defmethod ulx-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:ulx-val is deprecated.  Use NNTracker-msg:ulx instead.")
  (ulx m))

(cl:ensure-generic-function 'uly-val :lambda-list '(m))
(cl:defmethod uly-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:uly-val is deprecated.  Use NNTracker-msg:uly instead.")
  (uly m))

(cl:ensure-generic-function 'urx-val :lambda-list '(m))
(cl:defmethod urx-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:urx-val is deprecated.  Use NNTracker-msg:urx instead.")
  (urx m))

(cl:ensure-generic-function 'ury-val :lambda-list '(m))
(cl:defmethod ury-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:ury-val is deprecated.  Use NNTracker-msg:ury instead.")
  (ury m))

(cl:ensure-generic-function 'lrx-val :lambda-list '(m))
(cl:defmethod lrx-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:lrx-val is deprecated.  Use NNTracker-msg:lrx instead.")
  (lrx m))

(cl:ensure-generic-function 'lry-val :lambda-list '(m))
(cl:defmethod lry-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:lry-val is deprecated.  Use NNTracker-msg:lry instead.")
  (lry m))

(cl:ensure-generic-function 'llx-val :lambda-list '(m))
(cl:defmethod llx-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:llx-val is deprecated.  Use NNTracker-msg:llx instead.")
  (llx m))

(cl:ensure-generic-function 'lly-val :lambda-list '(m))
(cl:defmethod lly-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:lly-val is deprecated.  Use NNTracker-msg:lly instead.")
  (lly m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:area-val is deprecated.  Use NNTracker-msg:area instead.")
  (area m))

(cl:ensure-generic-function 'perimeter-val :lambda-list '(m))
(cl:defmethod perimeter-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:perimeter-val is deprecated.  Use NNTracker-msg:perimeter instead.")
  (perimeter m))

(cl:ensure-generic-function 'cmx-val :lambda-list '(m))
(cl:defmethod cmx-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:cmx-val is deprecated.  Use NNTracker-msg:cmx instead.")
  (cmx m))

(cl:ensure-generic-function 'cmy-val :lambda-list '(m))
(cl:defmethod cmy-val ((m <NNTrackerROI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NNTracker-msg:cmy-val is deprecated.  Use NNTracker-msg:cmy instead.")
  (cmy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NNTrackerROI>) ostream)
  "Serializes a message object of type '<NNTrackerROI>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ulx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uly))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'urx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ury))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lrx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'llx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lly))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'perimeter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NNTrackerROI>) istream)
  "Deserializes a message object of type '<NNTrackerROI>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ulx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uly) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'urx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ury) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lrx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lry) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'llx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lly) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'perimeter) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmy) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NNTrackerROI>)))
  "Returns string type for a message object of type '<NNTrackerROI>"
  "NNTracker/NNTrackerROI")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NNTrackerROI)))
  "Returns string type for a message object of type 'NNTrackerROI"
  "NNTracker/NNTrackerROI")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NNTrackerROI>)))
  "Returns md5sum for a message object of type '<NNTrackerROI>"
  "5f0574d8c71c1b4e4f70c00dc6bc1394")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NNTrackerROI)))
  "Returns md5sum for a message object of type 'NNTrackerROI"
  "5f0574d8c71c1b4e4f70c00dc6bc1394")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NNTrackerROI>)))
  "Returns full string definition for message of type '<NNTrackerROI>"
  (cl:format cl:nil "# Description of a warped rectangular region~%~%float32 ulx~%float32 uly~%~%float32 urx~%float32 ury~%~%float32 lrx~%float32 lry~%~%float32 llx~%float32 lly~%~%float32 area~%~%float32 perimeter~%~%float32 cmx~%float32 cmy~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NNTrackerROI)))
  "Returns full string definition for message of type 'NNTrackerROI"
  (cl:format cl:nil "# Description of a warped rectangular region~%~%float32 ulx~%float32 uly~%~%float32 urx~%float32 ury~%~%float32 lrx~%float32 lry~%~%float32 llx~%float32 lly~%~%float32 area~%~%float32 perimeter~%~%float32 cmx~%float32 cmy~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NNTrackerROI>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NNTrackerROI>))
  "Converts a ROS message object to a list"
  (cl:list 'NNTrackerROI
    (cl:cons ':ulx (ulx msg))
    (cl:cons ':uly (uly msg))
    (cl:cons ':urx (urx msg))
    (cl:cons ':ury (ury msg))
    (cl:cons ':lrx (lrx msg))
    (cl:cons ':lry (lry msg))
    (cl:cons ':llx (llx msg))
    (cl:cons ':lly (lly msg))
    (cl:cons ':area (area msg))
    (cl:cons ':perimeter (perimeter msg))
    (cl:cons ':cmx (cmx msg))
    (cl:cons ':cmy (cmy msg))
))
