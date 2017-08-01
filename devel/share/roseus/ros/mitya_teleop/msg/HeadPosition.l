;; Auto-generated. Do not edit!


(when (boundp 'mitya_teleop::HeadPosition)
  (if (not (find-package "MITYA_TELEOP"))
    (make-package "MITYA_TELEOP"))
  (shadow 'HeadPosition (find-package "MITYA_TELEOP")))
(unless (find-package "MITYA_TELEOP::HEADPOSITION")
  (make-package "MITYA_TELEOP::HEADPOSITION"))

(in-package "ROS")
;;//! \htmlinclude HeadPosition.msg.html


(defclass mitya_teleop::HeadPosition
  :super ros::object
  :slots (_horizontal _vertical ))

(defmethod mitya_teleop::HeadPosition
  (:init
   (&key
    ((:horizontal __horizontal) 0.0)
    ((:vertical __vertical) 0.0)
    )
   (send-super :init)
   (setq _horizontal (float __horizontal))
   (setq _vertical (float __vertical))
   self)
  (:horizontal
   (&optional __horizontal)
   (if __horizontal (setq _horizontal __horizontal)) _horizontal)
  (:vertical
   (&optional __vertical)
   (if __vertical (setq _vertical __vertical)) _vertical)
  (:serialization-length
   ()
   (+
    ;; float32 _horizontal
    4
    ;; float32 _vertical
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _horizontal
       (sys::poke _horizontal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vertical
       (sys::poke _vertical (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _horizontal
     (setq _horizontal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vertical
     (setq _vertical (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mitya_teleop::HeadPosition :md5sum-) "93817796a8d7b54c196b10312493371b")
(setf (get mitya_teleop::HeadPosition :datatype-) "mitya_teleop/HeadPosition")
(setf (get mitya_teleop::HeadPosition :definition-)
      "float32 horizontal
float32 vertical
")



(provide :mitya_teleop/HeadPosition "93817796a8d7b54c196b10312493371b")


