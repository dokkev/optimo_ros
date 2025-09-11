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
project(optimo_hardware)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ##############################################################################
# Set Package Config
# ##############################################################################

# To ignore EIGEN Warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

set(UNREL_OPTIMO_API_CMAKE
    "${CMAKE_INSTALL_PREFIX}/../optimo_api/share/optimo_api/cmake")
get_filename_component(OPTIMO_API_CMAKE "${UNREL_OPTIMO_API_CMAKE}" ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH ${OPTIMO_API_CMAKE})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(optimo_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(optimo_msgs REQUIRED)

set(ROS_PKGS rclcpp pluginlib hardware_interface optimo_api optimo_msgs)

# ##############################################################################
# Add and Export Library
# ##############################################################################

add_library(${PROJECT_NAME} SHARED src/OptimoHardwareInterface.cpp
                                   src/GripperHardwareInterface.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} ${ROS_PKGS})

pluginlib_export_plugin_description_file(hardware_interface optimo_hardware.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${ROS_PKGS})

ament_package()
