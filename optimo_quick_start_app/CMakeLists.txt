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
# CMake configuration
# ##############################################################################

cmake_minimum_required(VERSION 3.10)
project(optimo_quick_start_app)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build testing to true by default
option(BUILD_TESTING "Enable testing" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ##############################################################################
# Find dependencies
# ##############################################################################
# For FindYaml.cmake, FindEthercat.cmake Print the install directory Convert to
# absolute path

set(UNREL_OPTIMO_API_CMAKE
    "${CMAKE_INSTALL_PREFIX}/../optimo_api/share/optimo_api/cmake")
get_filename_component(OPTIMO_API_CMAKE "${UNREL_OPTIMO_API_CMAKE}" ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH ${OPTIMO_API_CMAKE})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(
  Qt5 5.15.3 EXACT
  COMPONENTS Core Gui Widgets Test
  REQUIRED)
find_package(optimo_api REQUIRED)

# setup qt depen
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ##############################################################################
# Setup app
# ##############################################################################

set(ROS_PKGS rclcpp std_srvs optimo_api)

set(QT_INCLUDE_DIRS ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS}
                    ${Qt5Widgets_INCLUDE_DIRS})

set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES}
                 ${Qt5Widgets_LIBRARIES})

set(APP_INCLUDE_DIRS ${QT_INCLUDE_DIRS})
set(APP_LIBRARIES ${QT_LIBRARIES})

set(GUI_HDRS
    # Core ###############################################################
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/core/optimo_quick_start_node.hpp
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/core/main_window.hpp
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/core/session.hpp
    # Views ###############################################################
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/views/debug_panel.hpp
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/views/jog_panel.hpp
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/views/traj_panel.hpp
    # Style ###############################################################
    ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/style/util.hpp)

set(GUI_SRC
    ${GUI_HDRS}
    # Core ###############################################################
    ${CMAKE_SOURCE_DIR}/src/core/optimo_quick_start_node.cpp
    ${CMAKE_SOURCE_DIR}/src/core/main_window.cpp
    ${CMAKE_SOURCE_DIR}/src/core/session.cpp
    # Views ###############################################################
    ${CMAKE_SOURCE_DIR}/src/views/debug_panel.cpp
    ${CMAKE_SOURCE_DIR}/src/views/jog_panel.cpp
    ${CMAKE_SOURCE_DIR}/src/views/traj_panel.cpp
    # Style ###############################################################
    ${CMAKE_SOURCE_DIR}/src/style/util.cpp)

set(GUI_HDR_DIRS ${CMAKE_SOURCE_DIR}/include)

# ##############################################################################
# Add executable
# ##############################################################################

add_executable(optimo_quick_start_app src/optimo_quick_start_app.cpp)
target_sources(optimo_quick_start_app PUBLIC ${GUI_SRC})
target_include_directories(optimo_quick_start_app PRIVATE ${GUI_HDR_DIRS}
                                                          ${APP_INCLUDE_DIRS})
target_link_libraries(optimo_quick_start_app ${APP_LIBRARIES})
ament_target_dependencies(optimo_quick_start_app ${ROS_PKGS})
target_compile_features(optimo_quick_start_app PUBLIC c_std_99 cxx_std_17)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ##############################################################################
# Linting
# ##############################################################################

if(BUILD_TESTING)
  # Checks Copyright and License in headers - run on save (check_header.sh)
  find_package(ament_cmake_copyright REQUIRED)

  # Linter for Code Changes (things like header-guards etc.) - manual fixes
  # required
  find_package(ament_cmake_cpplint REQUIRED)

  # Linter for non-code changes - vscode generic formatter with .clang-format
  find_package(ament_cmake_clang_format REQUIRED)

  # Linter - vscode generic formatter with
  find_package(ament_cmake_xmllint REQUIRED)

  # Python linter
  find_package(ament_cmake_flake8 REQUIRED)

  set(CPP_DIRECTORIES src include)
  ament_copyright()
  ament_cpplint()
  ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
  ament_xmllint()
  ament_flake8()

endif()

ament_package()
