;; Auto-generated. Do not edit!


(when (boundp 'interface::info)
  (if (not (find-package "INTERFACE"))
    (make-package "INTERFACE"))
  (shadow 'info (find-package "INTERFACE")))
(unless (find-package "INTERFACE::INFO")
  (make-package "INTERFACE::INFO"))

(in-package "ROS")
;;//! \htmlinclude info.msg.html


(defclass interface::info
  :super ros::object
  :slots ())

(defmethod interface::info
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get interface::info :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get interface::info :datatype-) "interface/info")
(setf (get interface::info :definition-)
      "
")



(provide :interface/info "d41d8cd98f00b204e9800998ecf8427e")


