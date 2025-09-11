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

#!/bin/bash

set -e
apt-get update && apt-get upgrade -y

# Clone SDK, build and install
cd /roboligent
git clone --depth 1 --single-branch https://$BITBUCKET_USERNAME:$BITBUCKET_APP_PASSWORD@bitbucket.org/roboligent/linkdyn_sdk.git
cd linkdyn_sdk
cmake . -B./build -DDOWNLOAD_RESOURCES=1 -DBUILD_SOURCE=0
cd build
make all -j$(nproc)
cd ..
rm -rf ./build ./bin
cmake . -B./build -DDOWNLOAD_RESOURCES=0 -DCMAKE_BUILD_TYPE=Release -DBUILD_SOURCE=1 -DBUILD_TESTS=1 -DINSTALL_SDK=1 -DBUILD_DOCS=1
cd build
make all -j$(nproc)
make install
cd /opt/atlassian/pipelines/agent/build

# Create ROS2 workspace
cd ..
mkdir -p ros2_ws/src/optimo_ros
cp -r build/. ros2_ws/src/optimo_ros/
cd ros2_ws

# Install rosdep
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths ./src/optimo_ros -y --ignore-src --rosdistro humble

# Build
colcon build --packages-up-to optimo_ros --parallel-workers $(nproc)
