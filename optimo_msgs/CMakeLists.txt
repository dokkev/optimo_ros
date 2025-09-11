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

cmake_minimum_required(VERSION 3.8)
project(optimo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/TrajectoryGoal.msg"
  "msg/TrajectoryParams.msg"
  "msg/ImpedanceParams.msg"
  "msg/PoseElbow.msg"
  "srv/PlayTrajCb.srv"
  "srv/StringCb.srv"
  "srv/GenericCb.srv"
  "srv/PosCb.srv"
  "srv/MoveitCb.srv"
  "srv/SetMode.srv"
  "srv/ServoCb.srv"
  DEPENDENCIES
  geometry_msgs)

ament_package()
