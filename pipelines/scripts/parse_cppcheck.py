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

import os
import sys
import json
import subprocess

# for debugging
# repo = "BITBUCKET_CONST"
# commit = "BITBUCKET_CONST"
# workspace = "BITBUCKET_CONST"
# branch = "feature/RS-782"

repo = os.environ["BITBUCKET_REPO_SLUG"]
commit = os.environ["BITBUCKET_COMMIT"]
workspace = os.environ["BITBUCKET_WORKSPACE"]
branch = os.environ["BITBUCKET_BRANCH"]
commit_url = "https://bitbucket.org/" + workspace + "/" + repo + "/commits/" + commit
all_annotations = []
git_diff_command = [
    "git",
    "diff",
    "--name-only",
    f"{branch}..remotes/origin/develop",
]
result = subprocess.run(git_diff_command, stdout=subprocess.PIPE, text=True)
modified_files_list = result.stdout.splitlines()


severities = {
    "error": "HIGH",
    "warning": "HIGH",
    "performance": "MEDIUM",
    "portability": "MEDIUM",
    "information": "LOW",
    "style": "LOW",
}

types = {
    "error": "BUG",
    "warning": "BUG",
    "performance": "VULNERABILITY",
    "portability": "VULNERABILITY",
    "information": "CODE_SMELL",
    "style": "CODE_SMELL",
}


def getResult(error_count):
    if error_count > 0:
        return "FAILED"
    elif is_file_modified(all_annotations, modified_files_list):
        return "FAILED"
    else:
        return "PASSED"


# Function to check if any file in annotations is modified
def is_file_modified(all_annotations, modified_files):
    for annotations in all_annotations:
        for annotation in annotations:
            # print(annotation["path"])
            if annotation["path"] in modified_files:
                return True
    return False


inputs = sys.argv
if len(inputs) != 2:
    print(
        "Input full file path (with trailing '/') for cppcheck output file. Files must be JSON-formatted!"
    )
else:
    input_file = inputs[1]

    with open(input_file) as cppcheck_output:
        file_title = "annotations"
        file_count = 0
        annotations_i = 0
        total_error_count = 0
        total_warning_count = 0
        annotations = []

        for entry in json.load(cppcheck_output)["annotations"]:
            annotations_i += 1
            if entry["severity"] == "error":
                total_error_count += 1
            elif entry["severity"] == "warning":
                total_warning_count += 1

            severity_str = str(entry["severity"])

            if len(entry["path"]) == 0:
                entry["path"] = "*"

            if len(entry["line"]) == 0:
                entry["line"] = "0"

            annotations.append(
                {
                    "external_id": "cppcheck_" + str(annotations_i),
                    "annotation_type": types[entry["severity"]],
                    "summary": severity_str.upper()
                    + " | "
                    + entry["id"]
                    + " | "
                    + entry["summary"],
                    "severity": severities[entry["severity"]],
                    "path": entry["path"],
                    "line": entry["line"],
                }
            )
            all_annotations.append(annotations)
            # Bitbucket Reports API handles 100 annotations max per POST request
            # New file every 100 annotations
            if annotations_i % 100 == 0:
                with open(
                    file_title + str(file_count) + ".json", "w"
                ) as annotation_file:
                    json.dump(annotations, annotation_file, indent=1)
                    file_count += 1
                    annotations = []

        with open(file_title + str(file_count) + ".json", "w") as annotation_file:
            json.dump(annotations, annotation_file, indent=1)
            file_count += 1

        with open("reports.json", "w") as report_file:
            report = {
                "title": "Cppcheck report",
                "details": "Cppcheck found "
                + str(annotations_i)
                + " issues with code quality!",
                "report_type": "SECURITY",
                "reporter": "cppcheck",
                "link": commit_url,
                "result": getResult(total_error_count),
                "data": [
                    {
                        "title": "Error Count",
                        "type": "NUMBER",
                        "value": total_error_count,
                    },
                    {
                        "title": "Warning Count",
                        "type": "NUMBER",
                        "value": total_warning_count,
                    },
                ],
            }
            # Write the report json to file
            json.dump(report, report_file, indent=4)

    # Get the list of modified files between HEAD and develop

    # print(git_diff_command)
    # print(result)
    # print(modified_files_list)
    # print(getResult(total_error_count))
    if getResult(total_error_count) == "FAILED":
        sys.exit(1)
    else:
        sys.exit(0)
