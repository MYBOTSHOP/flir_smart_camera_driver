cmake_minimum_required(VERSION 3.5)
project(flir_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

## Generate messages and services in their respective folders
set(msg_files
  "msg/TriggeredAlarms.msg"
  "msg/ActiveAlarms.msg"
  "msg/AlarmActions.msg"
  "msg/Lens.msg"
  "msg/Point.msg"
  "msg/ScreeningOperatorAction.msg"
)

set(srv_files
  "srv/FunctionDelta.srv"
  "srv/ImageState.srv"
  "srv/CalibrationRange.srv"
  "srv/FunctionLine.srv"
  "srv/RegionalTime.srv"
  "srv/DeviceConfiguration.srv"
  "srv/FunctionParameters.srv"
  "srv/RegionalUnits.srv"
  "srv/DeviceNuc.srv"
  "srv/DevicePan.srv"
  "srv/DeviceTilt.srv"
  "srv/FunctionPolyline.srv"
  "srv/ScreeningConfigurationLimits.srv"
  "srv/DigitalOutput.srv"
  "srv/FunctionRefTemp.srv"
  "srv/ScreeningConfiguration.srv"
  "srv/FunctionBox.srv"
  "srv/FunctionSpot.srv"
  "srv/ScreeningOperatorState.srv"
  "srv/DeviceDetails.srv"
  "srv/MeasureFunctionAlarm.srv"
  "srv/RegionalLimits.srv"
  "srv/DeviceFocusAutoUpdate.srv"
  "srv/DeviceFocusUpdate.srv"
  "srv/ScreeningOperatorConfiguration.srv"
  "srv/TempSensAlarm.srv"
  "srv/DiginAlarm.srv"
  "srv/RetrieveData.srv"
  "srv/IdRetrieveData.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
