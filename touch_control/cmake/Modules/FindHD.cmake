# Try to find the OpenHaptics HDAPI (3DSystems) libraries and headers.
#
# Once done this will define:
#
#   HD_FOUND         - System has HD
#   HD_INCLUDE_DIRS  - The HD include directories
#   HD_LIBRARIES     - The libraries needed to use HD

# Copyright (c) 2021, Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

find_path(HD_INCLUDE_DIR hd.h PATH_SUFFIXES HD)
find_library(HD_LIBRARY HD)

get_filename_component(OH_INCLUDE_DIR ${HD_INCLUDE_DIR} DIRECTORY)

set(HD_INCLUDE_DIRS ${OH_INCLUDE_DIR} ${HD_INCLUDE_DIR})
set(HD_LIBRARIES ${HD_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(HD DEFAULT_MSG HD_INCLUDE_DIR HD_LIBRARY)

mark_as_advanced(HD_INCLUDE_DIR HD_LIBRARY)

if(HD_FOUND AND NOT TARGET OpenHaptics::HD)
  add_library(OpenHaptics::HD UNKNOWN IMPORTED)
  set_target_properties(OpenHaptics::HD PROPERTIES
    IMPORTED_LOCATION "${HD_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${HD_INCLUDE_DIRS}")
endif()
