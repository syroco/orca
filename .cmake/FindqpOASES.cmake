# Copyright 2014 Istituto Italiano di Tecnologia (IIT)
#   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.

include(FindPackageHandleStandardArgs)

find_path(qpOASES_INCLUDEDIR
          NAMES qpOASES.hpp
          HINTS "${qpOASES_SOURCE_DIR}"
                ENV qpOASES_SOURCE_DIR
          PATH_SUFFIXES include)
find_library(qpOASES_LIB
             NAMES qpOASES
             HINTS "${qpOASES_BINARY_DIR}"
                   ENV qpOASES_BINARY_DIR
             PATH_SUFFIXES lib
                           libs)

set(qpOASES_INCLUDE_DIRS ${qpOASES_INCLUDEDIR})
set(qpOASES_LIBRARIES ${qpOASES_LIB})

find_package_handle_standard_args(qpOASES DEFAULT_MSG qpOASES_LIBRARIES
                                                      qpOASES_INCLUDE_DIRS)
set(qpOASES_FOUND ${QPOASES_FOUND})
