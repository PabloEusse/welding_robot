;; Auto-generated. Do not edit!


(when (boundp 'wsg_50_common::Incr)
  (if (not (find-package "WSG_50_COMMON"))
    (make-package "WSG_50_COMMON"))
  (shadow 'Incr (find-package "WSG_50_COMMON")))
(unless (find-package "WSG_50_COMMON::INCR")
  (make-package "WSG_50_COMMON::INCR"))
(unless (find-package "WSG_50_COMMON::INCRREQUEST")
  (make-package "WSG_50_COMMON::INCRREQUEST"))
(unless (find-package "WSG_50_COMMON::INCRRESPONSE")
  (make-package "WSG_50_COMMON::INCRRESPONSE"))

(in-package "ROS")





(defclass wsg_50_common::IncrRequest
  :super ros::object
  :slots (_direction _increment ))

(defmethod wsg_50_common::IncrRequest
  (:init
   (&key
    ((:direction __direction) "")
    ((:increment __increment) 0.0)
    )
   (send-super :init)
   (setq _direction (string __direction))
   (setq _increment (float __increment))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:increment
   (&optional __increment)
   (if __increment (setq _increment __increment)) _increment)
  (:serialization-length
   ()
   (+
    ;; string _direction
    4 (length _direction)
    ;; float32 _increment
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;; float32 _increment
       (sys::poke _increment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _increment
     (setq _increment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass wsg_50_common::IncrResponse
  :super ros::object
  :slots (_error ))

(defmethod wsg_50_common::IncrResponse
  (:init
   (&key
    ((:error __error) 0)
    )
   (send-super :init)
   (setq _error (round __error))
   self)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; uint8 _error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _error
       (write-byte _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass wsg_50_common::Incr
  :super ros::object
  :slots ())

(setf (get wsg_50_common::Incr :md5sum-) "b996830260cb95e784a9d9fbb5e34c61")
(setf (get wsg_50_common::Incr :datatype-) "wsg_50_common/Incr")
(setf (get wsg_50_common::Incr :request) wsg_50_common::IncrRequest)
(setf (get wsg_50_common::Incr :response) wsg_50_common::IncrResponse)

(defmethod wsg_50_common::IncrRequest
  (:response () (instance wsg_50_common::IncrResponse :init)))

(setf (get wsg_50_common::IncrRequest :md5sum-) "b996830260cb95e784a9d9fbb5e34c61")
(setf (get wsg_50_common::IncrRequest :datatype-) "wsg_50_common/IncrRequest")
(setf (get wsg_50_common::IncrRequest :definition-)
      "string direction
float32 increment
---
uint8 error

")

(setf (get wsg_50_common::IncrResponse :md5sum-) "b996830260cb95e784a9d9fbb5e34c61")
(setf (get wsg_50_common::IncrResponse :datatype-) "wsg_50_common/IncrResponse")
(setf (get wsg_50_common::IncrResponse :definition-)
      "string direction
float32 increment
---
uint8 error

")



(provide :wsg_50_common/Incr "b996830260cb95e784a9d9fbb5e34c61")


