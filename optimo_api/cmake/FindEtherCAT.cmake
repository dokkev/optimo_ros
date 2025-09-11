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

# This module defines ETHERCAT_LIBRARIES, the libraries to link against
# ETHERCAT_FOUND, if false, do not try to link to ETHERCAT ETHERCAT_INCLUDE_DIR,
# where to find headers

include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)

set(ETHERCAT_SEARCH_PATHS /usr/local/ ${ETHERCAT_SEARCH_PATHS})

find_path(
  ETHERCAT_INCLUDE_DIR
  NAMES ecrt.h
  HINTS
  PATH_SUFFIXES include/
  PATHS ${ETHERCAT_SEARCH_PATHS})

find_path(
  ETHERCAT_LIBRARY_DIR
  NAMES libethercat.so
  HINTS
  PATH_SUFFIXES lib/
  PATHS ${ETHERCAT_SEARCH_PATHS})

find_library(
  ETHERCAT_LIBRARY
  NAMES ethercat
  HINTS
  PATH_SUFFIXES lib/
  PATHS ${ETHERCAT_SEARCH_PATHS})

find_library(
  ETHERCAT_SNCN_LIBRARY
  NAMES ethercat_wrapper
  HINTS
  PATH_SUFFIXES lib/
  PATHS ${ETHERCAT_SEARCH_PATHS})

set(ETHERCAT_LIBRARIES ${ETHERCAT_SNCN_LIBRARY} ${ETHERCAT_LIBRARY})

mark_as_advanced(ETHERCAT_FOUND ETHERCAT_INCLUDE_DIR ETHERCAT_LIBRARIES)

find_package_handle_standard_args(EtherCAT REQUIRED_VAR ETHERCAT_INCLUDE_DIR
                                  ETHERCAT_LIBRARY_DIR ETHERCAT_LIBRARIES)
