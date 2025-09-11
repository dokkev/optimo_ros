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

if [ $# -eq 0 ]; then
  echo "Usage: $0 <prefix>"
  exit 1
fi

prefix=$1

# RUN TESTS
mkdir -p $PWD/gtest-reports
colcon test
colcon test-result --all
ctest_exit_code=$?
cp --recursive ./build/optimo_quick_start_app/test_results/optimo_quick_start_app/* ./gtest-reports/

# PUBLISH RESULTS
python3 ./src/optimo_ros/pipelines/scripts/parse_gtest_reports.py ./gtest-reports/ "${prefix}-gtest-report.json" "${prefix}"
python3 ./src/optimo_ros/pipelines/scripts/parse_gtest_annotations.py ./gtest-reports/ "${prefix}-gtest-report-annotations"

# SEND REPORTS
file_name="${prefix}-gtest-report.json"

echo $file_name
cat $file_name
curl -u $BITBUCKET_USERNAME:$BITBUCKET_APP_PASSWORD --request PUT "https://api.bitbucket.org/2.0/repositories/$BITBUCKET_WORKSPACE/$BITBUCKET_REPO_SLUG/commit/$BITBUCKET_COMMIT/reports/$prefix-gtest" --header 'Content-Type: application/json' --data-binary "@$file_name"

for file in "${prefix}-gtest-report-annotations"*.json
do
	echo -e "\nSending $file"
    cat $file
	curl -u $BITBUCKET_USERNAME:$BITBUCKET_APP_PASSWORD --request POST "https://api.bitbucket.org/2.0/repositories/$BITBUCKET_WORKSPACE/$BITBUCKET_REPO_SLUG/commit/$BITBUCKET_COMMIT/reports/$prefix-gtest/annotations" --header 'Content-Type: application/json' --data-binary "@$file"
done

if [ $ctest_exit_code -ne 0 ]; then
  exit 1
fi