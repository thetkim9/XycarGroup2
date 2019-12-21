
(cl:in-package :asdf)

(defsystem "YOLO-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "yoloMessage" :depends-on ("_package_yoloMessage"))
    (:file "_package_yoloMessage" :depends-on ("_package"))
  ))