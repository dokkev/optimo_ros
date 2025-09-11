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
import xml.etree.ElementTree as ET
import json
import re

serial_number = 0


def get_test_details(input_string):
    # Split the input string into lines
    lines = input_string.splitlines()

    if len(lines) >= 3:
        # return rest of the lines from 3-end of string
        return "\n".join(lines[2:])
    else:
        return ""


def extract_filename_and_line(input_string):
    # input_string = "/home/roboligent/CODE/linkdyn_sdk/test/utils/includes/TimerTest.h:123"

    # Define a regular expression pattern to match the file path and line number
    pattern = r"^(.*):(\d+)$"

    # Use re.match to find the match at the beginning of the string
    match = re.match(pattern, input_string)

    if match:
        # Extract the filename and line number from the match groups
        filename = match.group(1)
        line_number = int(match.group(2))
        current_directory = os.getcwd()
        filename = os.path.relpath(filename, current_directory)

        return filename, line_number
    else:
        # If there is no match, return None for both filename and line number
        return None, None


def parse_xml_and_generate_annotations(xml_folder, annotation_prefix):
    global serial_number
    annotation_count = 0
    annotation_file_index = 0
    annotations = []

    for filename in os.listdir(xml_folder):
        if filename.endswith(".xml"):
            xml_file = os.path.join(xml_folder, filename)
            tree = ET.parse(xml_file)
            root = tree.getroot()

            for testsuite in root.iter("testsuite"):
                for testcase in testsuite.iter("testcase"):
                    if testcase.find("failure") is not None:
                        annotation = generate_annotation(
                            testsuite, testcase, serial_number
                        )
                        annotations.append(annotation)
                        annotation_count += 1
                        serial_number += 1

                        if annotation_count >= 100:
                            annotation_filename = (
                                f"{annotation_prefix}{annotation_file_index}.json"
                            )
                            save_annotations(annotations, annotation_filename)
                            annotations = []
                            annotation_count = 0
                            annotation_file_index += 1

    # Save any remaining annotations
    if annotations:
        annotation_filename = f"{annotation_prefix}{annotation_file_index}.json"
        save_annotations(annotations, annotation_filename)


def generate_annotation(testsuite, testcase, serial_number):

    annotation = {
        "type": "Google Test Result",
        "external_id": f"gtest_{testsuite.get('name')}.{testcase.get('name')}_{serial_number}",
        "annotation_type": "BUG",
        "summary": f"{testsuite.get('name')}.{testcase.get('name')} Failed",
        "details": testcase.find("failure").get("message"),
        "result": "FAILED",
        "severity": "CRITICAL",
    }
    return annotation


def save_annotations(annotations, filename):
    with open(filename, "w") as outfile:
        json.dump(annotations, outfile, indent=4)


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 3:
        print("Usage: python script.py xml_folder annotation_file_prefix")
        sys.exit(1)

    xml_folder = sys.argv[1]
    annotation_prefix = sys.argv[2]

    if not os.path.exists(xml_folder):
        print(f"Folder '{xml_folder}' does not exist.")
        sys.exit(1)

    parse_xml_and_generate_annotations(xml_folder, annotation_prefix)
