;; Auto-generated. Do not edit!


(when (boundp 'YOLO::yoloMessage)
  (if (not (find-package "YOLO"))
    (make-package "YOLO"))
  (shadow 'yoloMessage (find-package "YOLO")))
(unless (find-package "YOLO::YOLOMESSAGE")
  (make-package "YOLO::YOLOMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude yoloMessage.msg.html


(defclass YOLO::yoloMessage
  :super ros::object
  :slots (_data ))

(defmethod YOLO::yoloMessage
  (:init
   (&key
    ((:data __data) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string[] _data
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _data i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get YOLO::yoloMessage :md5sum-) "cce5a364f3a3be12c9722c6dcad2fa94")
(setf (get YOLO::yoloMessage :datatype-) "YOLO/yoloMessage")
(setf (get YOLO::yoloMessage :definition-)
      "string[] data

")



(provide :YOLO/yoloMessage "cce5a364f3a3be12c9722c6dcad2fa94")


