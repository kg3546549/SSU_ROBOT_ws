; Auto-generated. Do not edit!


(cl:in-package mode_manager-srv)


;//! \htmlinclude ModeRequest-request.msg.html

(cl:defclass <ModeRequest-request> (roslisp-msg-protocol:ros-message)
  ((request_data
    :reader request_data
    :initarg :request_data
    :type cl:string
    :initform ""))
)

(cl:defclass ModeRequest-request (<ModeRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mode_manager-srv:<ModeRequest-request> is deprecated: use mode_manager-srv:ModeRequest-request instead.")))

(cl:ensure-generic-function 'request_data-val :lambda-list '(m))
(cl:defmethod request_data-val ((m <ModeRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mode_manager-srv:request_data-val is deprecated.  Use mode_manager-srv:request_data instead.")
  (request_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeRequest-request>) ostream)
  "Serializes a message object of type '<ModeRequest-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeRequest-request>) istream)
  "Deserializes a message object of type '<ModeRequest-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeRequest-request>)))
  "Returns string type for a service object of type '<ModeRequest-request>"
  "mode_manager/ModeRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeRequest-request)))
  "Returns string type for a service object of type 'ModeRequest-request"
  "mode_manager/ModeRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeRequest-request>)))
  "Returns md5sum for a message object of type '<ModeRequest-request>"
  "0c900dd8a9e0a09ef71b5a7480dbc006")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeRequest-request)))
  "Returns md5sum for a message object of type 'ModeRequest-request"
  "0c900dd8a9e0a09ef71b5a7480dbc006")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeRequest-request>)))
  "Returns full string definition for message of type '<ModeRequest-request>"
  (cl:format cl:nil "string request_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeRequest-request)))
  "Returns full string definition for message of type 'ModeRequest-request"
  (cl:format cl:nil "string request_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeRequest-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeRequest-request
    (cl:cons ':request_data (request_data msg))
))
;//! \htmlinclude ModeRequest-response.msg.html

(cl:defclass <ModeRequest-response> (roslisp-msg-protocol:ros-message)
  ((response_data
    :reader response_data
    :initarg :response_data
    :type cl:string
    :initform ""))
)

(cl:defclass ModeRequest-response (<ModeRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mode_manager-srv:<ModeRequest-response> is deprecated: use mode_manager-srv:ModeRequest-response instead.")))

(cl:ensure-generic-function 'response_data-val :lambda-list '(m))
(cl:defmethod response_data-val ((m <ModeRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mode_manager-srv:response_data-val is deprecated.  Use mode_manager-srv:response_data instead.")
  (response_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeRequest-response>) ostream)
  "Serializes a message object of type '<ModeRequest-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeRequest-response>) istream)
  "Deserializes a message object of type '<ModeRequest-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeRequest-response>)))
  "Returns string type for a service object of type '<ModeRequest-response>"
  "mode_manager/ModeRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeRequest-response)))
  "Returns string type for a service object of type 'ModeRequest-response"
  "mode_manager/ModeRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeRequest-response>)))
  "Returns md5sum for a message object of type '<ModeRequest-response>"
  "0c900dd8a9e0a09ef71b5a7480dbc006")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeRequest-response)))
  "Returns md5sum for a message object of type 'ModeRequest-response"
  "0c900dd8a9e0a09ef71b5a7480dbc006")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeRequest-response>)))
  "Returns full string definition for message of type '<ModeRequest-response>"
  (cl:format cl:nil "string response_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeRequest-response)))
  "Returns full string definition for message of type 'ModeRequest-response"
  (cl:format cl:nil "string response_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeRequest-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeRequest-response
    (cl:cons ':response_data (response_data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModeRequest)))
  'ModeRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModeRequest)))
  'ModeRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeRequest)))
  "Returns string type for a service object of type '<ModeRequest>"
  "mode_manager/ModeRequest")