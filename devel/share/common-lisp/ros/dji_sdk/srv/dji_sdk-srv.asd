
(cl:in-package :asdf)

(defsystem "dji_sdk-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :dji_sdk-msg
)
  :components ((:file "_package")
    (:file "Activation" :depends-on ("_package_Activation"))
    (:file "_package_Activation" :depends-on ("_package"))
    (:file "CameraAction" :depends-on ("_package_CameraAction"))
    (:file "_package_CameraAction" :depends-on ("_package"))
    (:file "DroneArmControl" :depends-on ("_package_DroneArmControl"))
    (:file "_package_DroneArmControl" :depends-on ("_package"))
    (:file "DroneTaskControl" :depends-on ("_package_DroneTaskControl"))
    (:file "_package_DroneTaskControl" :depends-on ("_package"))
    (:file "MFIOConfig" :depends-on ("_package_MFIOConfig"))
    (:file "_package_MFIOConfig" :depends-on ("_package"))
    (:file "MFIOSetValue" :depends-on ("_package_MFIOSetValue"))
    (:file "_package_MFIOSetValue" :depends-on ("_package"))
    (:file "MissionHpAction" :depends-on ("_package_MissionHpAction"))
    (:file "_package_MissionHpAction" :depends-on ("_package"))
    (:file "MissionHpGetInfo" :depends-on ("_package_MissionHpGetInfo"))
    (:file "_package_MissionHpGetInfo" :depends-on ("_package"))
    (:file "MissionHpResetYaw" :depends-on ("_package_MissionHpResetYaw"))
    (:file "_package_MissionHpResetYaw" :depends-on ("_package"))
    (:file "MissionHpUpdateRadius" :depends-on ("_package_MissionHpUpdateRadius"))
    (:file "_package_MissionHpUpdateRadius" :depends-on ("_package"))
    (:file "MissionHpUpdateYawRate" :depends-on ("_package_MissionHpUpdateYawRate"))
    (:file "_package_MissionHpUpdateYawRate" :depends-on ("_package"))
    (:file "MissionHpUpload" :depends-on ("_package_MissionHpUpload"))
    (:file "_package_MissionHpUpload" :depends-on ("_package"))
    (:file "MissionStatus" :depends-on ("_package_MissionStatus"))
    (:file "_package_MissionStatus" :depends-on ("_package"))
    (:file "MissionWpAction" :depends-on ("_package_MissionWpAction"))
    (:file "_package_MissionWpAction" :depends-on ("_package"))
    (:file "MissionWpGetInfo" :depends-on ("_package_MissionWpGetInfo"))
    (:file "_package_MissionWpGetInfo" :depends-on ("_package"))
    (:file "MissionWpGetSpeed" :depends-on ("_package_MissionWpGetSpeed"))
    (:file "_package_MissionWpGetSpeed" :depends-on ("_package"))
    (:file "MissionWpSetSpeed" :depends-on ("_package_MissionWpSetSpeed"))
    (:file "_package_MissionWpSetSpeed" :depends-on ("_package"))
    (:file "MissionWpUpload" :depends-on ("_package_MissionWpUpload"))
    (:file "_package_MissionWpUpload" :depends-on ("_package"))
    (:file "QueryDroneVersion" :depends-on ("_package_QueryDroneVersion"))
    (:file "_package_QueryDroneVersion" :depends-on ("_package"))
    (:file "SDKControlAuthority" :depends-on ("_package_SDKControlAuthority"))
    (:file "_package_SDKControlAuthority" :depends-on ("_package"))
    (:file "SendMobileData" :depends-on ("_package_SendMobileData"))
    (:file "_package_SendMobileData" :depends-on ("_package"))
    (:file "SendPayloadData" :depends-on ("_package_SendPayloadData"))
    (:file "_package_SendPayloadData" :depends-on ("_package"))
    (:file "SetHardSync" :depends-on ("_package_SetHardSync"))
    (:file "_package_SetHardSync" :depends-on ("_package"))
    (:file "SetLocalPosRef" :depends-on ("_package_SetLocalPosRef"))
    (:file "_package_SetLocalPosRef" :depends-on ("_package"))
    (:file "SetupCameraStream" :depends-on ("_package_SetupCameraStream"))
    (:file "_package_SetupCameraStream" :depends-on ("_package"))
    (:file "Stereo240pSubscription" :depends-on ("_package_Stereo240pSubscription"))
    (:file "_package_Stereo240pSubscription" :depends-on ("_package"))
    (:file "StereoDepthSubscription" :depends-on ("_package_StereoDepthSubscription"))
    (:file "_package_StereoDepthSubscription" :depends-on ("_package"))
    (:file "StereoVGASubscription" :depends-on ("_package_StereoVGASubscription"))
    (:file "_package_StereoVGASubscription" :depends-on ("_package"))
  ))