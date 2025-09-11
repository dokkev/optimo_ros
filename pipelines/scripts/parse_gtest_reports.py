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
import xml.etree.ElementTree as ET


def process_xml_file(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    tests = int(root.get("tests", 0))
    failures = int(root.get("failures", 0))

    return tests, failures


def generate_json_report(total_tests, total_passed, total_failed, commit_url):
    report = {
        "title": f"{prefix}-Google Test Report",
        "details": f"Google Test Found {total_failed} test failures out of {total_tests} Test Cases!",
        "report_type": "TEST",
        "reporter": "Google Test",
        "link": commit_url,
        "result": "FAILED" if total_failed > 0 else "PASSED",
        "data": [
            {"title": "Total Test:", "type": "NUMBER", "value": total_tests},
            {"title": "Passed:", "type": "NUMBER", "value": total_passed},
            {"title": "Failed:", "type": "NUMBER", "value": total_failed},
        ],
    }

    return report


def main(folder_path, commit_url, output_file):
    total_tests = 0
    total_passed = 0
    total_failed = 0

    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".xml"):
                xml_file = os.path.join(root, file)
                tests, failures = process_xml_file(xml_file)
                total_tests += tests
                total_failed += failures
                total_passed += tests - failures

    report = generate_json_report(total_tests, total_passed, total_failed, commit_url)

    with open(output_file, "w") as output_file:
        json.dump(report, output_file, indent=4)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python script.py folder_path output_file report_title_prefix")
        sys.exit(1)

    folder_path = sys.argv[1]
    output_file = sys.argv[2]
    prefix = sys.argv[3]

    if not os.path.exists(folder_path):
        print(f"Folder '{folder_path}' does not exist.")
        sys.exit(1)

    repo = os.environ.get("BITBUCKET_REPO_SLUG", "")
    commit = os.environ.get("BITBUCKET_COMMIT", "")
    workspace = os.environ.get("BITBUCKET_WORKSPACE", "")
    commit_url = (
        "https://bitbucket.org/" + workspace + "/" + repo + "/commits/" + commit
    )

    main(folder_path, commit_url, output_file)
