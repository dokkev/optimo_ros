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


# GENERATE CPPCHECK.OUT
echo "" > ./cppcheck.out
echo -e "{\n\t\"annotations\": [\n" >> ./cppcheck.out

# Get all ROS2 packages in this repo
ros2_packages=$(colcon list --names-only)

# Create a string of include paths for all ROS2 packages
include_paths=""
for package in $ros2_packages; do
    if [ -d "./$package/include" ]; then
        include_paths+=" -I ./$package/include"
    fi
done

cppcheck \
  --template="\t{\n\
    \t\t\"summary\": \"{message}\",\n\
    \t\t\"id\": \"{id}\",\n\
    \t\t\"severity\": \"{severity}\",\n\
    \t\t\"path\": \"{file}\",\n\
    \t\t\"line\": \"{line}\"\n\
    \t}," \
  --language=c++ \
  --enable=all \
  --inconclusive \
  --std=c++17 \
  --force \
  --suppressions-list=./pipelines/dependencies/cppcheck-suppress.txt \
  --check-config \
  $include_paths \
  ./*/src \
  2>> ./cppcheck.out


truncate -s-2 ./cppcheck.out
echo -e "\n\t]\n}" >> ./cppcheck.out

python3 ./pipelines/scripts/prep_cppcheck_output.py ./cppcheck.out

# GENERATE REPORTS.JSON, ANNOTATIONS.JSON
echo -e "\nParsing cppcheck.out"
python3 ./pipelines/scripts/parse_cppcheck.py ./cppcheck.out
CPPCHECK_ERROR=$?
echo "$CPPCHECK_ERROR"
cat reports.json

# SEND REPORTS
echo -e "\nSending report"
curl -u $BITBUCKET_USERNAME:$BITBUCKET_APP_PASSWORD --request PUT "https://api.bitbucket.org/2.0/repositories/$BITBUCKET_WORKSPACE/$BITBUCKET_REPO_SLUG/commit/$BITBUCKET_COMMIT/reports/cppcheck" --header 'Content-Type: application/json' --data @reports.json

for file in annotations*.json
do
	echo -e "\nSending $file"
	curl -u $BITBUCKET_USERNAME:$BITBUCKET_APP_PASSWORD --request POST "https://api.bitbucket.org/2.0/repositories/$BITBUCKET_WORKSPACE/$BITBUCKET_REPO_SLUG/commit/$BITBUCKET_COMMIT/reports/cppcheck/annotations" --header 'Content-Type: application/json' --data @$file
done

exit $CPPCHECK_ERROR
