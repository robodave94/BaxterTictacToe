; Auto-generated. Do not edit!


(cl:in-package baxter_collaboration_msgs-msg)


;//! \htmlinclude ObjectsArray.msg.html

(cl:defclass <ObjectsArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector baxter_collaboration_msgs-msg:Object)
   :initform (cl:make-array 0 :element-type 'baxter_collaboration_msgs-msg:Object :initial-element (cl:make-instance 'baxter_collaboration_msgs-msg:Object))))
)

(cl:defclass ObjectsArray (<ObjectsArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectsArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectsArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name baxter_collaboration_msgs-msg:<ObjectsArray> is deprecated: use baxter_collaboration_msgs-msg:ObjectsArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader baxter_collaboration_msgs-msg:header-val is deprecated.  Use baxter_collaboration_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <ObjectsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader baxter_collaboration_msgs-msg:objects-val is deprecated.  Use baxter_collaboration_msgs-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectsArray>) ostream)
  "Serializes a message object of type '<ObjectsArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectsArray>) istream)
  "Deserializes a message object of type '<ObjectsArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'baxter_collaboration_msgs-msg:Object))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectsArray>)))
  "Returns string type for a message object of type '<ObjectsArray>"
  "baxter_collaboration_msgs/ObjectsArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectsArray)))
  "Returns string type for a message object of type 'ObjectsArray"
  "baxter_collaboration_msgs/ObjectsArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectsArray>)))
  "Returns md5sum for a message object of type '<ObjectsArray>"
  "e866a9d87ce3e4706204628f251a2f44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectsArray)))
  "Returns md5sum for a message object of type 'ObjectsArray"
  "e866a9d87ce3e4706204628f251a2f44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectsArray>)))
  "Returns full string definition for message of type '<ObjectsArray>"
  (cl:format cl:nil "Header                             header~%baxter_collaboration_msgs/Object[] objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: baxter_collaboration_msgs/Object~%~%uint32              id~%string              name~%geometry_msgs/Pose  pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectsArray)))
  "Returns full string definition for message of type 'ObjectsArray"
  (cl:format cl:nil "Header                             header~%baxter_collaboration_msgs/Object[] objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: baxter_collaboration_msgs/Object~%~%uint32              id~%string              name~%geometry_msgs/Pose  pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectsArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectsArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectsArray
    (cl:cons ':header (header msg))
    (cl:cons ':objects (objects msg))
))
