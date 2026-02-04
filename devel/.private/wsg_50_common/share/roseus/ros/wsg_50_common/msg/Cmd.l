;; Auto-generated. Do not edit!


(when (boundp 'wsg_50_common::Cmd)
  (if (not (find-package "WSG_50_COMMON"))
    (make-package "WSG_50_COMMON"))
  (shadow 'Cmd (find-package "WSG_50_COMMON")))
(unless (find-package "WSG_50_COMMON::CMD")
  (make-package "WSG_50_COMMON::CMD"))

(in-package "ROS")
;;//! \htmlinclude Cmd.msg.html


(defclass wsg_50_common::Cmd
  :super ros::object
  :slots (_mode _pos _speed ))

(defmethod wsg_50_common::Cmd
  (:init
   (&key
    ((:mode __mode) "")
    ((:pos __pos) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _mode (string __mode))
   (setq _pos (float __pos))
   (setq _speed (float __speed))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ;; float32 _pos
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; float32 _pos
       (sys::poke _pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _pos
     (setq _pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wsg_50_common::Cmd :md5sum-) "a9d4654d92a2e086717420189f98c76a")
(setf (get wsg_50_common::Cmd :datatype-) "wsg_50_common/Cmd")
(setf (get wsg_50_common::Cmd :definition-)
      "string mode
float32 pos
float32 speed


")



(provide :wsg_50_common/Cmd "a9d4654d92a2e086717420189f98c76a")


