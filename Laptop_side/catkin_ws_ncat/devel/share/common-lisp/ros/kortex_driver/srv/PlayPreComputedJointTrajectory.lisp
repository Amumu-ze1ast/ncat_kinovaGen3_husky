; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlayPreComputedJointTrajectory-request.msg.html

(cl:defclass <PlayPreComputedJointTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:PreComputedJointTrajectory
    :initform (cl:make-instance 'kortex_driver-msg:PreComputedJointTrajectory)))
)

(cl:defclass PlayPreComputedJointTrajectory-request (<PlayPreComputedJointTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayPreComputedJointTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayPreComputedJointTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayPreComputedJointTrajectory-request> is deprecated: use kortex_driver-srv:PlayPreComputedJointTrajectory-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlayPreComputedJointTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayPreComputedJointTrajectory-request>) ostream)
  "Serializes a message object of type '<PlayPreComputedJointTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayPreComputedJointTrajectory-request>) istream)
  "Deserializes a message object of type '<PlayPreComputedJointTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayPreComputedJointTrajectory-request>)))
  "Returns string type for a service object of type '<PlayPreComputedJointTrajectory-request>"
  "kortex_driver/PlayPreComputedJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayPreComputedJointTrajectory-request)))
  "Returns string type for a service object of type 'PlayPreComputedJointTrajectory-request"
  "kortex_driver/PlayPreComputedJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayPreComputedJointTrajectory-request>)))
  "Returns md5sum for a message object of type '<PlayPreComputedJointTrajectory-request>"
  "50902897eedd6708728c63b8108c9da3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayPreComputedJointTrajectory-request)))
  "Returns md5sum for a message object of type 'PlayPreComputedJointTrajectory-request"
  "50902897eedd6708728c63b8108c9da3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayPreComputedJointTrajectory-request>)))
  "Returns full string definition for message of type '<PlayPreComputedJointTrajectory-request>"
  (cl:format cl:nil "PreComputedJointTrajectory input~%~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectory~%~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayPreComputedJointTrajectory-request)))
  "Returns full string definition for message of type 'PlayPreComputedJointTrajectory-request"
  (cl:format cl:nil "PreComputedJointTrajectory input~%~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectory~%~%uint32 mode~%PreComputedJointTrajectoryElement[] trajectory_elements~%================================================================================~%MSG: kortex_driver/PreComputedJointTrajectoryElement~%~%float32[] joint_angles~%float32[] joint_speeds~%float32[] joint_accelerations~%float32 time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayPreComputedJointTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayPreComputedJointTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayPreComputedJointTrajectory-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlayPreComputedJointTrajectory-response.msg.html

(cl:defclass <PlayPreComputedJointTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlayPreComputedJointTrajectory-response (<PlayPreComputedJointTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayPreComputedJointTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayPreComputedJointTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayPreComputedJointTrajectory-response> is deprecated: use kortex_driver-srv:PlayPreComputedJointTrajectory-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlayPreComputedJointTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayPreComputedJointTrajectory-response>) ostream)
  "Serializes a message object of type '<PlayPreComputedJointTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayPreComputedJointTrajectory-response>) istream)
  "Deserializes a message object of type '<PlayPreComputedJointTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayPreComputedJointTrajectory-response>)))
  "Returns string type for a service object of type '<PlayPreComputedJointTrajectory-response>"
  "kortex_driver/PlayPreComputedJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayPreComputedJointTrajectory-response)))
  "Returns string type for a service object of type 'PlayPreComputedJointTrajectory-response"
  "kortex_driver/PlayPreComputedJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayPreComputedJointTrajectory-response>)))
  "Returns md5sum for a message object of type '<PlayPreComputedJointTrajectory-response>"
  "50902897eedd6708728c63b8108c9da3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayPreComputedJointTrajectory-response)))
  "Returns md5sum for a message object of type 'PlayPreComputedJointTrajectory-response"
  "50902897eedd6708728c63b8108c9da3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayPreComputedJointTrajectory-response>)))
  "Returns full string definition for message of type '<PlayPreComputedJointTrajectory-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayPreComputedJointTrajectory-response)))
  "Returns full string definition for message of type 'PlayPreComputedJointTrajectory-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayPreComputedJointTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayPreComputedJointTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayPreComputedJointTrajectory-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayPreComputedJointTrajectory)))
  'PlayPreComputedJointTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayPreComputedJointTrajectory)))
  'PlayPreComputedJointTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayPreComputedJointTrajectory)))
  "Returns string type for a service object of type '<PlayPreComputedJointTrajectory>"
  "kortex_driver/PlayPreComputedJointTrajectory")