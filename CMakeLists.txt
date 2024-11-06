cmake_minimum_required(VERSION 3.16)
project(rv2_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Check ROS_DISTRO
set(MY_ROS2_DISTRO $ENV{ROS_DISTRO})
if(${MY_ROS2_DISTRO} STREQUAL "foxy")
  add_definitions(-DFOUND_FOXY)
elseif(${MY_ROS2_DISTRO} STREQUAL "humble")
  add_definitions(-DFOUND_HUMBLE)
else()
  message(FATAL_ERROR "ROS_DISTRO is not found or not supported.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



set(msg_content_files
  "msg_content/DevInfo.msg"
  "msg_content/DevManageServerStatus.msg"
  "msg_content/InteractiveService.msg"
  "msg_content/InteractiveServiceMasterPrivilege.msg"
  "msg_content/NodeAddr.msg"
  "msg_content/ProcStatus.msg"
  "msg_content/QosProfile.msg"
  "msg_content/QosServerStatus.msg"
  "msg_content/TopicDeviceInfo.msg"
  "msg_content/TopicQosProfile.msg"
)

set(srv_files
  "srv/DevInfoReq.srv"
  "srv/DevManageServer.srv"
  "srv/InteractiveService.srv"
  "srv/InteractiveServiceReg.srv"
  "srv/InteractiveServiceReq.srv"
  "srv/NodeAddrReg.srv"
  "srv/NodeAddrReq.srv"
  "srv/ProcStatusReg.srv"
  "srv/ProcStatusReq.srv"
  "srv/QosServer.srv"
  "srv/TopicDeviceInfoReg.srv"
  "srv/TopicQosProfileReg.srv"
  "srv/TopicQosProfileReq.srv"
  )

SET(PYLIB False CACHE BOOL "Some user-specified option")

if( PYLIB ) # PYLIB

find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)
find_package(${PROJECT_NAME} REQUIRED)

set(src_files
 "src/pybind11.cpp"
)

ament_python_install_package(${PROJECT_NAME})

add_library(cpplib MODULE ${src_files})
ament_target_dependencies(cpplib rclcpp ${PROJECT_NAME} pybind11)
# pybind11_add_module(cpplib ${src_files})

set_target_properties(cpplib PROPERTIES PREFIX "")

if(${MY_ROS2_DISTRO} STREQUAL "foxy")
  install(
    TARGETS cpplib
    DESTINATION lib/python3.8/site-packages/${PROJECT_NAME}
  )
elseif(${MY_ROS2_DISTRO} STREQUAL "humble")
  install(
    TARGETS cpplib
    DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )
endif()

else() # NOT PYLIB

## Build
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_content_files} ${srv_files} DEPENDENCIES builtin_interfaces sensor_msgs)


## Install
ament_export_dependencies(rosidl_default_runtime)

if(${MY_ROS2_DISTRO} STREQUAL "foxy")
  install(
    DIRECTORY include/
    DESTINATION include
  )
  ament_export_include_directories(
    include
  )
  install(
    FILES requirements_apt.txt
    DESTINATION share/${PROJECT_NAME}
  )
elseif(${MY_ROS2_DISTRO} STREQUAL "humble")
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
  ament_export_include_directories(
    include
  )
  install(
    FILES requirements_apt.txt
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()

endif() # if(PYLIB)
