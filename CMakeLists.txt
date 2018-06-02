cmake_minimum_required(VERSION 3.1)
project(orca VERSION 2.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CMakePackageConfigHelpers)

#################### Eigen3 ###########################
find_package(Eigen3 3.2.92 QUIET)

if(NOT Eigen3_FOUND)
    if(EIGEN3_INCLUDE_DIR)
        unset(EIGEN3_INCLUDE_DIR CACHE)
    endif()
    message(WARNING "Using internal Eigen3 (3.3.90), please consider installing it standalone.")
    set(BUILD_TESTING OFF CACHE BOOL "Disable eigen tests")
    # NOTE: Adding the eigen project allows all the checks for processor features
    # and system environnement libraries.
    # It might be useful to add this on a very particular system to get some
    # very specific processor features and link agains the right BLAS for example.
    # Ex of an issue in a Xenomai env : http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1482
    # Some linking issues might happend, which are fixed with the 'auto' config
    # of eigen subdir. This is considered necessary for the code to be portable
    add_subdirectory(external/eigen-3.3.90/eigen3)
    set(ENV{EIGEN3_ROOT} ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90)
    set(EIGEN3_ROOTDIR $ENV{EIGEN3_ROOT})
    set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90/eigen3)
endif()

#################### TinyXML ###########################
set(tinyxml_SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/idyntree-0.8.1/extern/tinyxml/src)
include_directories(${tinyxml_SOURCE_DIR})
add_library(${PROJECT_NAME}_priv_tinyxml STATIC
    ${tinyxml_SOURCE_DIR}/tinyxml.cpp
    ${tinyxml_SOURCE_DIR}/tinyxmlerror.cpp
    ${tinyxml_SOURCE_DIR}/tinyxmlparser.cpp
)

#################### qpOASES ##########################
set(qpOASES_INCLUDE_DIRS external/qpOASES-3.2.1/include )
file(GLOB qpOASES_srcs ${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES-3.2.1/src/*.cpp)

add_library(${PROJECT_NAME}_priv_qpOASES STATIC ${qpOASES_srcs})
target_compile_definitions(${PROJECT_NAME}_priv_qpOASES PRIVATE __NO_COPYRIGHT__)
target_include_directories(${PROJECT_NAME}_priv_qpOASES PRIVATE external/qpOASES-3.2.1/include)

#################### iDynTree #########################
set(iDynTree_INCLUDE_DIRS
    external/idyntree-0.8.1/src/core/include
    external/idyntree-0.8.1/src/estimation/include
    external/idyntree-0.8.1/src/high-level/include
    external/idyntree-0.8.1/src/icub/include
    external/idyntree-0.8.1/src/inverse-kinematics/include
    external/idyntree-0.8.1/src/model/include
    external/idyntree-0.8.1/src/model_io/iKin-kdl/include
    external/idyntree-0.8.1/src/model_io/symoro/include
    external/idyntree-0.8.1/src/model_io/urdf/include
    external/idyntree-0.8.1/src/model_io/urdf-kdl/include
    external/idyntree-0.8.1/src/regressors/include
    external/idyntree-0.8.1/src/sensors/include
    external/idyntree-0.8.1/src/visualization/include
    external/idyntree-0.8.1/src/yarp/include
)

set(iDynTree_LIBRARIES_ALL
    idyntree-core
    idyntree-estimation
    idyntree-high-level
    idyntree-icub
    idyntree-inverse-kinematics
    idyntree-model
    idyntree-modelio-urdf
    idyntree-modelio-ikin-kdl
    idyntree-modelio-symoro
    idyntree-modelio-urdf-kdl
    idyntree-regressors
    idyntree-sensors
    idyntree-visualization
    idyntree-yarp
)

#set(IDYNTREE_SHARED_LIBRARY FALSE CACHE BOOL "Build iDynTree as a shared library")
add_subdirectory(external/idyntree-0.8.1)

foreach(l : ${iDynTree_LIBRARIES_ALL})
    if(TARGET ${l})
        list(APPEND iDynTree_LIBRARIES ${l})
    endif()
endforeach()

add_library(iDynTree::idyntree-high-level ALIAS idyntree-high-level)

#################### Catkin ###########################

find_package(catkin QUIET)
if(catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS include #${EIGEN3_INCLUDE_DIR} ${iDynTree_INCLUDE_DIRS} external/qpOASES-3.2.1/include external/plog-1.1.3/include external/json-2.1.1/include
        LIBRARIES orca #${iDynTree_LIBRARIES}
    )
endif()

if(catkin_FOUND AND NOT iDynTree_FOUND)
    foreach(l : ${iDynTree_LIBRARIES})
    if(TARGET ${l})
        set(lib_name ${CMAKE_SHARED_LIBRARY_PREFIX}${l}${CMAKE_SHARED_LIBRARY_SUFFIX})
        add_custom_target(  ${l}-targets-copy ALL
                            DEPENDS ${l}
                            COMMAND ${CMAKE_COMMAND} -E copy
                            ${CMAKE_CURRENT_BINARY_DIR}/lib/${lib_name}
                            ${CATKIN_DEVEL_PREFIX}/lib/${lib_name}
                            COMMENT "Copying ${CMAKE_CURRENT_BINARY_DIR}/lib/${lib_name} to ${CATKIN_DEVEL_PREFIX}/lib/${lib_name}"
                         )
        endif()
    endforeach()
endif()

#################### ORCA #########################
set(sub-dirs
    optim
    constraint
    common
    math
    robot
    task
    utils
)

foreach(sub : ${sub-dirs})
    file(GLOB sub-srcs src/${sub}/*.cc)
    list(APPEND lib-srcs ${sub-srcs})
endforeach()

source_group("Source Files" FILES ${lib-srcs})

foreach(sub : ${sub-dirs})
    file(GLOB sub-hdrs include/orca/${sub}/*.h include/orca/${sub}/*.hh include/orca/${sub}/*.impl)
    list(APPEND lib-hdrs ${sub-hdrs})
endforeach()

source_group("Header Files" FILES ${lib-hdrs})

add_library(orca ${lib-srcs})
target_compile_features(orca PUBLIC
    cxx_auto_type
    cxx_generalized_initializers
)

target_include_directories(
      orca PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

if(NOT Eigen3_FOUND)
    target_include_directories(
          orca PUBLIC
          $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
          $<INSTALL_INTERFACE:include/eigen3>
      )
else()
    target_include_directories(orca PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_include_directories(orca PRIVATE ${qpOASES_INCLUDE_DIRS})

target_include_directories(
      orca PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/plog-1.1.4/include>
      $<INSTALL_INTERFACE:include>
  )

find_package(PythonLibs QUIET)
find_package(NumPy QUIET)
if(PYTHONLIBS_FOUND AND NUMPY_FOUND)
    target_include_directories(
          orca PUBLIC
          $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/matplotlibcpp/include>
          $<INSTALL_INTERFACE:include>
      )
    target_link_libraries(orca PUBLIC ${PYTHON_LIBRARIES})
    install(DIRECTORY external/matplotlibcpp/include/matplotlibcpp DESTINATION include/matplotlibcpp)
endif()

target_link_libraries(orca PRIVATE iDynTree::idyntree-high-level)
target_link_libraries(orca PRIVATE ${PROJECT_NAME}_priv_qpOASES)
target_link_libraries(orca PRIVATE ${PROJECT_NAME}_priv_tinyxml)

#################### ORCA Examples #########################
add_subdirectory(examples)

#################### ORCA Tests #########################
add_subdirectory(tests)

#################### Linter #########################
add_custom_target(  ${PROJECT_NAME}-compile-commands ALL
                    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                    COMMAND ${CMAKE_COMMAND} -E copy
                    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                    ${PROJECT_SOURCE_DIR}
                    COMMENT "Copying 'compile_commands.json' to '${PROJECT_SOURCE_DIR}'"
                 )

#################### Doxygen #########################
# TODO: This code would be useful as a standalone module for pure doxygen builds
# FIXME: We are using RTD to build the doxygen now. Remove this once, the module has been made.
# option(BUILD_API_DOC "Build documentation" OFF)
#
# if(BUILD_API_DOC)
#     # check if Doxygen is installed
#     find_package(Doxygen QUIET)
#     if (DOXYGEN_FOUND)
#         # set input and output files
#         set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
#         set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#
#         # request to configure the file
#         configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
#         message("Doxygen build started")
#
#         add_custom_target( doc_doxygen ALL
#             COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#             COMMENT "Generating API documentation with Doxygen"
#             VERBATIM )
#     else (DOXYGEN_FOUND)
#       message(WARNING "Doxygen needs to be installed to generate the doxygen documentation")
#     endif (DOXYGEN_FOUND)
# endif(BUILD_API_DOC)

#################### ORCA Install #########################
install(TARGETS orca DESTINATION lib EXPORT orcaTargets)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hh"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE
)

install(DIRECTORY external/plog-1.1.4/include/plog DESTINATION include)

#
# Cmake Package
#
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/orcaConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

set(CONFIG_PACKAGE_LOCATION share/cmake/orca)
set(INCLUDE_INSTALL_DIR include/ )

configure_package_config_file(cmake/orcaConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
    INSTALL_DESTINATION ${CONFIG_PACKAGE_LOCATION}
    PATH_VARS INCLUDE_INSTALL_DIR
)

export(EXPORT orcaTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake"
    NAMESPACE orca::
)

export(PACKAGE orca)
# Export iDynTree library
#export(PACKAGE iDynTree)

if(catkin_FOUND)
    add_custom_target(  ${PROJECT_NAME}-targets-copy ALL
                        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake
                        COMMAND ${CMAKE_COMMAND} -E copy
                        ${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake
                        ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/orcaTargets.cmake
                        COMMENT "Copying 'orcaTargets.cmake' to '${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake'"
                     )
    add_custom_target(  ${PROJECT_NAME}-config-copy ALL
                        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
                        COMMAND ${CMAKE_COMMAND} -E copy
                        ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
                        ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/orcaConfig.cmake
                        COMMENT "Copying 'orcaConfig.cmake' to '${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake'"
                     )
    if(NOT iDynTree_FOUND)
        add_custom_target(  idyntree-targets-copy ALL
                            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/external/idyntree-0.8.1/iDynTreeTargets.cmake
                            COMMAND ${CMAKE_COMMAND} -E copy
                            ${CMAKE_CURRENT_BINARY_DIR}/external/idyntree-0.8.1/iDynTreeTargets.cmake
                            ${CATKIN_DEVEL_PREFIX}/share/iDynTree/cmake/iDynTreeTargets.cmake
                            COMMENT "Copying 'iDynTreeTargets.cmake' to '${CATKIN_DEVEL_PREFIX}/share/iDynTree/cmake'"
                         )
        add_custom_target(  idyntree-config-copy ALL
                            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/external/idyntree-0.8.1/iDynTreeConfig.cmake
                            COMMAND ${CMAKE_COMMAND} -E copy
                            ${CMAKE_CURRENT_BINARY_DIR}/external/idyntree-0.8.1/iDynTreeConfig.cmake
                            ${CATKIN_DEVEL_PREFIX}/share/iDynTree/cmake/iDynTreeConfig.cmake
                            COMMENT "Copying 'iDynTreeConfig.cmake' to '${CATKIN_DEVEL_PREFIX}/share/iDynTree/cmake'"
                         )
    endif()
endif()


install(EXPORT orcaTargets
    FILE orcaTargets.cmake
    NAMESPACE orca::
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/orcaConfigVersion.cmake
    ${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
)
