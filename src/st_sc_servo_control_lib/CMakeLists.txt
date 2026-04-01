cmake_minimum_required(VERSION 3.8)
project(st_sc_servo_control_lib)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/SCS.cpp
  src/SCSCL.cpp
  src/SCSerial.cpp
  src/SMSBL.cpp
  src/SMSCL.cpp
  src/SMS_STS.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT st_sc_servo_control_libTargets
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/st_sc_servo_control_lib/
  DESTINATION include/st_sc_servo_control_lib
)

ament_export_targets(st_sc_servo_control_libTargets HAS_LIBRARY_TARGET)

ament_package()
