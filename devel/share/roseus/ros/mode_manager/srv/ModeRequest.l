;; Auto-generated. Do not edit!


(when (boundp 'mode_manager::ModeRequest)
  (if (not (find-package "MODE_MANAGER"))
    (make-package "MODE_MANAGER"))
  (shadow 'ModeRequest (find-package "MODE_MANAGER")))
(unless (find-package "MODE_MANAGER::MODEREQUEST")
  (make-package "MODE_MANAGER::MODEREQUEST"))
(unless (find-package "MODE_MANAGER::MODEREQUESTREQUEST")
  (make-package "MODE_MANAGER::MODEREQUESTREQUEST"))
(unless (find-package "MODE_MANAGER::MODEREQUESTRESPONSE")
  (make-package "MODE_MANAGER::MODEREQUESTRESPONSE"))

(in-package "ROS")





(defclass mode_manager::ModeRequestRequest
  :super ros::object
  :slots (_request_data ))

(defmethod mode_manager::ModeRequestRequest
  (:init
   (&key
    ((:request_data __request_data) "")
    )
   (send-super :init)
   (setq _request_data (string __request_data))
   self)
  (:request_data
   (&optional __request_data)
   (if __request_data (setq _request_data __request_data)) _request_data)
  (:serialization-length
   ()
   (+
    ;; string _request_data
    4 (length _request_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _request_data
       (write-long (length _request_data) s) (princ _request_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _request_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _request_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mode_manager::ModeRequestResponse
  :super ros::object
  :slots (_response_data ))

(defmethod mode_manager::ModeRequestResponse
  (:init
   (&key
    ((:response_data __response_data) "")
    )
   (send-super :init)
   (setq _response_data (string __response_data))
   self)
  (:response_data
   (&optional __response_data)
   (if __response_data (setq _response_data __response_data)) _response_data)
  (:serialization-length
   ()
   (+
    ;; string _response_data
    4 (length _response_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response_data
       (write-long (length _response_data) s) (princ _response_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mode_manager::ModeRequest
  :super ros::object
  :slots ())

(setf (get mode_manager::ModeRequest :md5sum-) "0c900dd8a9e0a09ef71b5a7480dbc006")
(setf (get mode_manager::ModeRequest :datatype-) "mode_manager/ModeRequest")
(setf (get mode_manager::ModeRequest :request) mode_manager::ModeRequestRequest)
(setf (get mode_manager::ModeRequest :response) mode_manager::ModeRequestResponse)

(defmethod mode_manager::ModeRequestRequest
  (:response () (instance mode_manager::ModeRequestResponse :init)))

(setf (get mode_manager::ModeRequestRequest :md5sum-) "0c900dd8a9e0a09ef71b5a7480dbc006")
(setf (get mode_manager::ModeRequestRequest :datatype-) "mode_manager/ModeRequestRequest")
(setf (get mode_manager::ModeRequestRequest :definition-)
      "string request_data
---
string response_data
")

(setf (get mode_manager::ModeRequestResponse :md5sum-) "0c900dd8a9e0a09ef71b5a7480dbc006")
(setf (get mode_manager::ModeRequestResponse :datatype-) "mode_manager/ModeRequestResponse")
(setf (get mode_manager::ModeRequestResponse :definition-)
      "string request_data
---
string response_data
")



(provide :mode_manager/ModeRequest "0c900dd8a9e0a09ef71b5a7480dbc006")


