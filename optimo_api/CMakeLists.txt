# Copyright 2024 Roboligent, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# ##############################################################################
# CONFIGURATION
# ##############################################################################

cmake_minimum_required(VERSION 3.8)
project(optimo_api)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Configure the resource header file
configure_file(${CMAKE_SOURCE_DIR}/resources/Resource.h.in
               ${CMAKE_SOURCE_DIR}/include/optimo_api/Resource.h @ONLY)

# To ignore EIGEN Warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# ##############################################################################
# Set Package Config
# ##############################################################################

# find dependencies
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(optimo_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Roboligent 1.0.5 EXACT REQUIRED)
find_package(srdfdom REQUIRED)

set(ROS_PKGS rclcpp std_msgs sensor_msgs moveit_ros_planning_interface
             optimo_msgs)
set(API_PKGS Roboligent)
# needed becuase ros messes with the default include paths
set(ROBOLIGENT_SDK_INCLUDE_DIRS "/usr/local/include/roboligent-1.0.5")
# Adding Packages to Build Includes
set(APP_INCLUDE_DIRS ${ROBOLIGENT_SDK_INCLUDE_DIRS})
# Adding Packages to Build Libraries
set(APP_LIBRARIES ${ROBOLIGENT_SDK_LIBRARIES})

# ##############################################################################
# Optimo API
# ##############################################################################

set(CNTR_SRC
    ${CMAKE_SOURCE_DIR}/src/Controller.cpp
    ${CMAKE_SOURCE_DIR}/src/IdleCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/FreeMotionCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/CalibrationCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/RecordCalibrationCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/CallbackCommander.cpp
    ${CMAKE_SOURCE_DIR}/src/Task.cpp
    ${CMAKE_SOURCE_DIR}/src/JogCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/PlayTrajCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/TeachCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/MoveHomeCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/MoveItCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/MoveItServoCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/MoveItInterface.cpp
    ${CMAKE_SOURCE_DIR}/src/ServoCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/ServoFbCallback.cpp
    ${CMAKE_SOURCE_DIR}/src/EEServoCallback.cpp)

set(CNTR_HDR_DIRS ${CMAKE_SOURCE_DIR}/include)

add_library(optimo_api SHARED ${CNTR_SRC})
target_include_directories(
  optimo_api
  PUBLIC ${APP_INCLUDE_DIRS}
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>"
         ${srdfdom_INCLUDE_DIRS})

target_link_libraries(optimo_api ${APP_LIBRARIES}
		      ${srdfdom_LIBRARIES})
ament_target_dependencies(optimo_api ${ROS_PKGS})

install(DIRECTORY resources cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS optimo_api
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS_PKGS} ${API_PKGS})
ament_export_include_directories(include)

ament_package()
