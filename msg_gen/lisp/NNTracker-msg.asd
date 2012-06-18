
(cl:in-package :asdf)

(defsystem "NNTracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NNTrackerROI" :depends-on ("_package_NNTrackerROI"))
    (:file "_package_NNTrackerROI" :depends-on ("_package"))
  ))