;; Auto-generated. Do not edit!


(when (boundp 'wsg_50_common::Conf)
  (if (not (find-package "WSG_50_COMMON"))
    (make-package "WSG_50_COMMON"))
  (shadow 'Conf (find-package "WSG_50_COMMON")))
(unless (find-package "WSG_50_COMMON::CONF")
  (make-package "WSG_50_COMMON::CONF"))
(unless (find-package "WSG_50_COMMON::CONFREQUEST")
  (make-package "WSG_50_COMMON::CONFREQUEST"))
(unless (find-package "WSG_50_COMMON::CONFRESPONSE")
  (make-package "WSG_50_COMMON::CONFRESPONSE"))

(in-package "ROS")





(defclass wsg_50_common::ConfRequest
  :super ros::object
  :slots (_val ))

(defmethod wsg_50_common::ConfRequest
  (:init
   (&key
    ((:val __val) 0.0)
    )
   (send-super :init)
   (setq _val (float __val))
   self)
  (:val
   (&optional __val)
   (if __val (setq _val __val)) _val)
  (:serialization-length
   ()
   (+
    ;; float32 _val
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _val
       (sys::poke _val (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _val
     (setq _val (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass wsg_50_common::ConfResponse
  :super ros::object
  :slots (_error ))

(defmethod wsg_50_common::ConfResponse
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

(defclass wsg_50_common::Conf
  :super ros::object
  :slots ())

(setf (get wsg_50_common::Conf :md5sum-) "0c7a1839200830facdc1205cf36615d5")
(setf (get wsg_50_common::Conf :datatype-) "wsg_50_common/Conf")
(setf (get wsg_50_common::Conf :request) wsg_50_common::ConfRequest)
(setf (get wsg_50_common::Conf :response) wsg_50_common::ConfResponse)

(defmethod wsg_50_common::ConfRequest
  (:response () (instance wsg_50_common::ConfResponse :init)))

(setf (get wsg_50_common::ConfRequest :md5sum-) "0c7a1839200830facdc1205cf36615d5")
(setf (get wsg_50_common::ConfRequest :datatype-) "wsg_50_common/ConfRequest")
(setf (get wsg_50_common::ConfRequest :definition-)
      "float32 val
---
uint8 error

")

(setf (get wsg_50_common::ConfResponse :md5sum-) "0c7a1839200830facdc1205cf36615d5")
(setf (get wsg_50_common::ConfResponse :datatype-) "wsg_50_common/ConfResponse")
(setf (get wsg_50_common::ConfResponse :definition-)
      "float32 val
---
uint8 error

")



(provide :wsg_50_common/Conf "0c7a1839200830facdc1205cf36615d5")


