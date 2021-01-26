;; Auto-generated. Do not edit!


(when (boundp 'mcuserial_msgs::RequestParam)
  (if (not (find-package "MCUSERIAL_MSGS"))
    (make-package "MCUSERIAL_MSGS"))
  (shadow 'RequestParam (find-package "MCUSERIAL_MSGS")))
(unless (find-package "MCUSERIAL_MSGS::REQUESTPARAM")
  (make-package "MCUSERIAL_MSGS::REQUESTPARAM"))
(unless (find-package "MCUSERIAL_MSGS::REQUESTPARAMREQUEST")
  (make-package "MCUSERIAL_MSGS::REQUESTPARAMREQUEST"))
(unless (find-package "MCUSERIAL_MSGS::REQUESTPARAMRESPONSE")
  (make-package "MCUSERIAL_MSGS::REQUESTPARAMRESPONSE"))

(in-package "ROS")





(defclass mcuserial_msgs::RequestParamRequest
  :super ros::object
  :slots (_name ))

(defmethod mcuserial_msgs::RequestParamRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mcuserial_msgs::RequestParamResponse
  :super ros::object
  :slots (_ints _floats _strings ))

(defmethod mcuserial_msgs::RequestParamResponse
  (:init
   (&key
    ((:ints __ints) (make-array 0 :initial-element 0 :element-type :integer))
    ((:floats __floats) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:strings __strings) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _ints __ints)
   (setq _floats __floats)
   (setq _strings __strings)
   self)
  (:ints
   (&optional __ints)
   (if __ints (setq _ints __ints)) _ints)
  (:floats
   (&optional __floats)
   (if __floats (setq _floats __floats)) _floats)
  (:strings
   (&optional __strings)
   (if __strings (setq _strings __strings)) _strings)
  (:serialization-length
   ()
   (+
    ;; int32[] _ints
    (* 4    (length _ints)) 4
    ;; float32[] _floats
    (* 4    (length _floats)) 4
    ;; string[] _strings
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _strings)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _ints
     (write-long (length _ints) s)
     (dotimes (i (length _ints))
       (write-long (elt _ints i) s)
       )
     ;; float32[] _floats
     (write-long (length _floats) s)
     (dotimes (i (length _floats))
       (sys::poke (elt _floats i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; string[] _strings
     (write-long (length _strings) s)
     (dolist (elem _strings)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _ints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ints (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ints i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _floats
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _floats (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _floats i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; string[] _strings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _strings (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _strings i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass mcuserial_msgs::RequestParam
  :super ros::object
  :slots ())

(setf (get mcuserial_msgs::RequestParam :md5sum-) "d7a0c2be00c9fd03cc69f2863de9c4d9")
(setf (get mcuserial_msgs::RequestParam :datatype-) "mcuserial_msgs/RequestParam")
(setf (get mcuserial_msgs::RequestParam :request) mcuserial_msgs::RequestParamRequest)
(setf (get mcuserial_msgs::RequestParam :response) mcuserial_msgs::RequestParamResponse)

(defmethod mcuserial_msgs::RequestParamRequest
  (:response () (instance mcuserial_msgs::RequestParamResponse :init)))

(setf (get mcuserial_msgs::RequestParamRequest :md5sum-) "d7a0c2be00c9fd03cc69f2863de9c4d9")
(setf (get mcuserial_msgs::RequestParamRequest :datatype-) "mcuserial_msgs/RequestParamRequest")
(setf (get mcuserial_msgs::RequestParamRequest :definition-)
      "string name

---

int32[]   ints
float32[] floats
string[]  strings

")

(setf (get mcuserial_msgs::RequestParamResponse :md5sum-) "d7a0c2be00c9fd03cc69f2863de9c4d9")
(setf (get mcuserial_msgs::RequestParamResponse :datatype-) "mcuserial_msgs/RequestParamResponse")
(setf (get mcuserial_msgs::RequestParamResponse :definition-)
      "string name

---

int32[]   ints
float32[] floats
string[]  strings

")



(provide :mcuserial_msgs/RequestParam "d7a0c2be00c9fd03cc69f2863de9c4d9")


