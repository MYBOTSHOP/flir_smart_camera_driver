cmake_minimum_required(VERSION 3.0.2)
project(flir_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TriggeredAlarms.msg
  ActiveAlarms.msg
  AlarmActions.msg
  Lens.msg
  Point.msg
  ScreeningOperatorAction.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FunctionDelta.srv
  ImageState.srv
  CalibrationRange.srv
  FunctionLine.srv
  RegionalTime.srv
  DeviceConfiguration.srv
  FunctionParameters.srv
  RegionalUnits.srv
  DeviceNuc.srv
  DevicePan.srv
  DeviceTilt.srv
  FunctionPolyline.srv
  ScreeningConfigurationLimits.srv
  DigitalOutput.srv
  FunctionRefTemp.srv
  ScreeningConfiguration.srv
  FunctionBox.srv
  FunctionSpot.srv
  ScreeningOperatorState.srv
  DeviceDetails.srv
  MeasureFunctionAlarm.srv
  RegionalLimits.srv
  DeviceFocusAutoUpdate.srv
  DeviceFocusUpdate.srv
  ScreeningOperatorConfiguration.srv
  TempSensAlarm.srv
  DiginAlarm.srv
  RetrieveData.srv
  IdRetrieveData.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)