;; Auto-generated. Do not edit!


(when (boundp 'wsg_50_common::Move)
  (if (not (find-package "WSG_50_COMMON"))
    (make-package "WSG_50_COMMON"))
  (shadow 'Move (find-package "WSG_50_COMMON")))
(unless (find-package "WSG_50_COMMON::MOVE")
  (make-package "WSG_50_COMMON::MOVE"))
(unless (find-package "WSG_50_COMMON::MOVEREQUEST")
  (make-package "WSG_50_COMMON::MOVEREQUEST"))
(unless (find-package "WSG_50_COMMON::MOVERESPONSE")
  (make-package "WSG_50_COMMON::MOVERESPONSE"))

(in-package "ROS")





(defclass wsg_50_common::MoveRequest
  :super ros::object
  :slots (_width _speed ))

(defmethod wsg_50_common::MoveRequest
  (:init
   (&key
    ((:width __width) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _width (float __width))
   (setq _speed (float __speed))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _width
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass wsg_50_common::MoveResponse
  :super ros::object
  :slots (_error ))

(defmethod wsg_50_common::MoveResponse
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

(defclass wsg_50_common::Move
  :super ros::object
  :slots ())

(setf (get wsg_50_common::Move :md5sum-) "ac4bc9278b95c49c6bb95b3db7e0a0e4")
(setf (get wsg_50_common::Move :datatype-) "wsg_50_common/Move")
(setf (get wsg_50_common::Move :request) wsg_50_common::MoveRequest)
(setf (get wsg_50_common::Move :response) wsg_50_common::MoveResponse)

(defmethod wsg_50_common::MoveRequest
  (:response () (instance wsg_50_common::MoveResponse :init)))

(setf (get wsg_50_common::MoveRequest :md5sum-) "ac4bc9278b95c49c6bb95b3db7e0a0e4")
(setf (get wsg_50_common::MoveRequest :datatype-) "wsg_50_common/MoveRequest")
(setf (get wsg_50_common::MoveRequest :definition-)
      "float32 width
float32 speed
---
uint8 error

")

(setf (get wsg_50_common::MoveResponse :md5sum-) "ac4bc9278b95c49c6bb95b3db7e0a0e4")
(setf (get wsg_50_common::MoveResponse :datatype-) "wsg_50_common/MoveResponse")
(setf (get wsg_50_common::MoveResponse :definition-)
      "float32 width
float32 speed
---
uint8 error

")



(provide :wsg_50_common/Move "ac4bc9278b95c49c6bb95b3db7e0a0e4")


