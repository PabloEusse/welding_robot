;; Auto-generated. Do not edit!


(when (boundp 'wsg_50_common::Status)
  (if (not (find-package "WSG_50_COMMON"))
    (make-package "WSG_50_COMMON"))
  (shadow 'Status (find-package "WSG_50_COMMON")))
(unless (find-package "WSG_50_COMMON::STATUS")
  (make-package "WSG_50_COMMON::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html


(defclass wsg_50_common::Status
  :super ros::object
  :slots (_status _width _speed _acc _force _force_finger0 _force_finger1 ))

(defmethod wsg_50_common::Status
  (:init
   (&key
    ((:status __status) "")
    ((:width __width) 0.0)
    ((:speed __speed) 0.0)
    ((:acc __acc) 0.0)
    ((:force __force) 0.0)
    ((:force_finger0 __force_finger0) 0.0)
    ((:force_finger1 __force_finger1) 0.0)
    )
   (send-super :init)
   (setq _status (string __status))
   (setq _width (float __width))
   (setq _speed (float __speed))
   (setq _acc (float __acc))
   (setq _force (float __force))
   (setq _force_finger0 (float __force_finger0))
   (setq _force_finger1 (float __force_finger1))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:force_finger0
   (&optional __force_finger0)
   (if __force_finger0 (setq _force_finger0 __force_finger0)) _force_finger0)
  (:force_finger1
   (&optional __force_finger1)
   (if __force_finger1 (setq _force_finger1 __force_finger1)) _force_finger1)
  (:serialization-length
   ()
   (+
    ;; string _status
    4 (length _status)
    ;; float32 _width
    4
    ;; float32 _speed
    4
    ;; float32 _acc
    4
    ;; float32 _force
    4
    ;; float32 _force_finger0
    4
    ;; float32 _force_finger1
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force
       (sys::poke _force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force_finger0
       (sys::poke _force_finger0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force_finger1
       (sys::poke _force_finger1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acc
     (setq _acc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force
     (setq _force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force_finger0
     (setq _force_finger0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force_finger1
     (setq _force_finger1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wsg_50_common::Status :md5sum-) "51c764be2e2c02863274a528dd8b3470")
(setf (get wsg_50_common::Status :datatype-) "wsg_50_common/Status")
(setf (get wsg_50_common::Status :definition-)
      "string status
float32 width
float32 speed
float32 acc
float32 force
float32 force_finger0
float32 force_finger1

")



(provide :wsg_50_common/Status "51c764be2e2c02863274a528dd8b3470")


