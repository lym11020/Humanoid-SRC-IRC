
(cl:in-package :asdf)

(defsystem "ahra-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Emergency" :depends-on ("_package_Emergency"))
    (:file "_package_Emergency" :depends-on ("_package"))
    (:file "RL_NeckAngle" :depends-on ("_package_RL_NeckAngle"))
    (:file "_package_RL_NeckAngle" :depends-on ("_package"))
    (:file "Select_Motion" :depends-on ("_package_Select_Motion"))
    (:file "_package_Select_Motion" :depends-on ("_package"))
    (:file "SendMotion" :depends-on ("_package_SendMotion"))
    (:file "_package_SendMotion" :depends-on ("_package"))
    (:file "Turn_Angle" :depends-on ("_package_Turn_Angle"))
    (:file "_package_Turn_Angle" :depends-on ("_package"))
    (:file "UD_NeckAngle" :depends-on ("_package_UD_NeckAngle"))
    (:file "_package_UD_NeckAngle" :depends-on ("_package"))
  ))