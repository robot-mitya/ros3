;; Auto-generated. Do not edit!


(when (boundp 'mitya_teleop::Drive)
  (if (not (find-package "MITYA_TELEOP"))
    (make-package "MITYA_TELEOP"))
  (shadow 'Drive (find-package "MITYA_TELEOP")))
(unless (find-package "MITYA_TELEOP::DRIVE")
  (make-package "MITYA_TELEOP::DRIVE"))

(in-package "ROS")
;;//! \htmlinclude Drive.msg.html


(defclass mitya_teleop::Drive
  :super ros::object
  :slots (_left _right ))

(defmethod mitya_teleop::Drive
  (:init
   (&key
    ((:left __left) 0)
    ((:right __right) 0)
    )
   (send-super :init)
   (setq _left (round __left))
   (setq _right (round __right))
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; int8 _left
    1
    ;; int8 _right
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _left
       (write-byte _left s)
     ;; int8 _right
       (write-byte _right s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _left
     (setq _left (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _left 127) (setq _left (- _left 256)))
   ;; int8 _right
     (setq _right (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _right 127) (setq _right (- _right 256)))
   ;;
   self)
  )

(setf (get mitya_teleop::Drive :md5sum-) "24825b8956c21f4c3dd28a5a4d09322c")
(setf (get mitya_teleop::Drive :datatype-) "mitya_teleop/Drive")
(setf (get mitya_teleop::Drive :definition-)
      "int8 left
int8 right

")



(provide :mitya_teleop/Drive "24825b8956c21f4c3dd28a5a4d09322c")


