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

# This module defines YAML_LIBRARY, the libraries to link against YAML_FOUND, if
# false, do not try to link to ETHERCAT YAML_INCLUDE_DIR, where to find headers

include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)

set(YAML_SEARCH_PATHS /usr/ /usr/local ${YAML_SEARCH_PATHS})

find_path(
  YAML_INCLUDE_DIR
  NAMES yaml.h
  HINTS
  PATH_SUFFIXES include/ include/yaml
  PATHS ${YAML_SEARCH_PATHS})

find_path(
  YAML_LIBRARY_DIR
  NAMES libyaml
  HINTS
  PATH_SUFFIXES lib/
  PATHS ${YAML_SEARCH_PATHS})

find_library(
  YAML_LIBRARY
  NAMES yaml
  HINTS
  PATH_SUFFIXES lib/
  PATHS ${YAML_SEARCH_PATHS})

mark_as_advanced(YAML_FOUND YAML_INCLUDE_DIR YAML_LIBRARY)

find_package_handle_standard_args(Yaml REQUIRED_VAR YAML_INCLUDE_DIR
                                  YAML_LIBRARY)
