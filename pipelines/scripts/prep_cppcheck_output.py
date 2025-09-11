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


def find_lines_with_quotes(file_path):
    with open(file_path, "r") as file:
        lines = file.readlines()

    with open(file_path, "w") as file:
        for line_number, line in enumerate(lines, start=1):
            quote_indices = [i for i, char in enumerate(line) if char == '"']
            if len(quote_indices) > 4:
                # Keep the 1st, 2nd, 3rd, and last instance of '"'
                keep_indices = set(quote_indices[:3] + [quote_indices[-1]])
                new_line = "".join(
                    char if i not in quote_indices or i in keep_indices else ""
                    for i, char in enumerate(line)
                )
                if new_line != line:
                    print(f"Line {line_number} changed: {new_line.strip()}")
                line = new_line
            file.write(line)


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python prep_cppcheck_output.py <file_path>")
    else:
        find_lines_with_quotes(sys.argv[1])
