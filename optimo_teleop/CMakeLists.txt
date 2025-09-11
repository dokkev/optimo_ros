# Copyright 2024 Roboligent, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(optimo_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(UNREL_OPTIMO_API_CMAKE
    "${CMAKE_INSTALL_PREFIX}/../optimo_api/share/optimo_api/cmake")
get_filename_component(OPTIMO_API_CMAKE "${UNREL_OPTIMO_API_CMAKE}" ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH ${OPTIMO_API_CMAKE})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(optimo_api REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(optimo_msgs REQUIRED)

# Build teleop_node executable
add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp std_msgs sensor_msgs optimo_msgs optimo_api)
target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build twist_converter executable
add_executable(twist_converter src/twist_converter.cpp)
ament_target_dependencies(twist_converter rclcpp std_msgs sensor_msgs optimo_msgs optimo_api)
target_include_directories(twist_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  teleop_node
  twist_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
