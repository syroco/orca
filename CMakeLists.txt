cmake_minimum_required(VERSION 3.1)
project(orca VERSION 2.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CMakePackageConfigHelpers)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#################### Xenomai ###########################
find_package(Xenomai QUIET)

if(XENOMAI_FOUND)
    message(STATUS "[ORCA] Building with Xenomai mutexes")
endif()

#################### Eigen3 ###########################
if(EIGEN3_INCLUDE_DIR)
    unset(EIGEN3_INCLUDE_DIR CACHE)
endif()
if(EIGEN3_VERSION)
    unset(EIGEN3_VERSION CACHE)
endif()

find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    set(system_eigen3_include_dir ${EIGEN3_INCLUDE_DIR})
    set(system_eigen3_version ${EIGEN3_VERSION})
endif()

# iDynTree requires Eigen > 3.2
find_package(Eigen3 3.2.92 QUIET)
if(Eigen3_FOUND)
    message("Eigen3 ${system_eigen3_version} (> 3.2) found at ${system_eigen3_include_dir}")
else()
    if(EIGEN3_INCLUDE_DIR)
        unset(EIGEN3_INCLUDE_DIR CACHE)
    endif()
    if(EIGEN3_VERSION)
        unset(EIGEN3_VERSION CACHE)
    endif()

    # NOTE: Adding the eigen project allows all the checks for processor features
    # and system environnement libraries.
    # It might be useful to add this on a very particular system to get some
    # very specific processor features and link agains the right BLAS for example.
    # Ex of an issue in a Xenomai env : http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1482
    # Some linking issues might happend, which are fixed with the 'auto' config
    # of eigen subdir. This is considered necessary for the code to be portable
    set(BUILD_TESTING OFF CACHE BOOL "Disable eigen tests")
    if(system_eigen3_found)
        message(WARNING "Found system Eigen3 ${system_eigen3_version} at ${system_eigen3_include_dir}, but internal iDynTree requires at least 3.2.
        ORCA will build using internal Eigen3 (3.3.90) for the build but will not overwrite the system version.")
        # it means we have it on the system, just not the right version
        # so we dont mess with the system libs
        add_subdirectory(external/eigen-3.3.90/eigen3 EXCLUDE_FROM_ALL)
    else()
        message(WARNING "Using internal Eigen3 (3.3.90) for the build. ORCA will export it as the system does not seem to have eigen3 installed")
        # export/install the package as we really dont have eigen on the system
        add_subdirectory(external/eigen-3.3.90/eigen3)
    endif()
    set(ENV{EIGEN3_ROOT} ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90)
    set(EIGEN3_ROOTDIR $ENV{EIGEN3_ROOT})
    set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90/eigen3)
    add_library(Eigen3::Eigen ALIAS eigen)
endif()

#################### TinyXML ###########################
set(tinyxml_SOURCE_DIR external/idyntree-0.8.1/extern/tinyxml/src)
add_library(${PROJECT_NAME}_priv_tinyxml STATIC
    ${tinyxml_SOURCE_DIR}/tinyxml.cpp
    ${tinyxml_SOURCE_DIR}/tinyxmlerror.cpp
    ${tinyxml_SOURCE_DIR}/tinyxmlparser.cpp
)
target_include_directories(${PROJECT_NAME}_priv_tinyxml PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/${tinyxml_SOURCE_DIR}>)

#################### qpOASES ##########################
set(qpOASES_INCLUDE_DIRS external/qpOASES-3.2.1/include)
file(GLOB qpOASES_srcs ${PROJECT_SOURCE_DIR}/external/qpOASES-3.2.1/src/*.cpp)

add_library(${PROJECT_NAME}_priv_qpOASES STATIC ${qpOASES_srcs})
target_compile_definitions(${PROJECT_NAME}_priv_qpOASES PRIVATE __NO_COPYRIGHT__)
target_include_directories(${PROJECT_NAME}_priv_qpOASES PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/qpOASES-3.2.1/include>)

#################### OSQP #########################
set(UNITTESTS OFF CACHE BOOL "")
set(PROFILING OFF CACHE BOOL "")
set(ENABLE_MKL_PARDISO OFF CACHE BOOL "")
set(DLONG OFF CACHE BOOL "Use int instead of long long for storing the indexes to match eigen sparseviews")
add_subdirectory(external/osqp-0.3.0 EXCLUDE_FROM_ALL)

#################### iDynTree #########################
if(NOT BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
endif()
set(build_shared_libs_saved ${BUILD_SHARED_LIBS})

set(BUILD_SHARED_LIBS OFF)
set(IDYNTREE_SHARED_LIBRARY OFF CACHE BOOL "")
add_subdirectory(${PROJECT_SOURCE_DIR}/external/idyntree-0.8.1 EXCLUDE_FROM_ALL)
set(BUILD_SHARED_LIBS ${build_shared_libs_saved})

#################### Catkin ###########################
find_package(catkin QUIET)

#################### Matplotlib #########################
find_package(PythonLibs QUIET)
find_package(NumPy QUIET)
if(PYTHONLIBS_FOUND AND NUMPY_FOUND)
    set(matplotlibcpp_include_dirs ${PROJECT_SOURCE_DIR}/external/matplotlibcpp/include)
endif()

#################### Catkin package ###########################
if(catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${matplotlibcpp_include_dirs} external/plog-1.1.4/include
        LIBRARIES orca ${PYTHON_LIBRARIES}
        CFG_EXTRAS orca-extras.cmake
    )
else()
    set(CATKIN_GLOBAL_BIN_DESTINATION bin)
    set(CATKIN_GLOBAL_LIB_DESTINATION lib)
    set(CATKIN_GLOBAL_LIBEXEC_DESTINATION lib)
    set(CATKIN_GLOBAL_INCLUDE_DESTINATION include)
    set(CATKIN_PACKAGE_LIB_DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
    set(CATKIN_GLOBAL_SHARE_DESTINATION share)
    set(CATKIN_PACKAGE_SHARE_DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME})
    set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME})
    set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})
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
    file(GLOB sub-hdrs include/orca/${sub}/*.h include/orca/${sub}/*.hh)
    list(APPEND lib-hdrs ${sub-hdrs})
endforeach()

source_group("Header Files" FILES ${lib-hdrs})

add_library(orca ${lib-srcs})
target_compile_features(orca PUBLIC
    cxx_auto_type
    cxx_generalized_initializers
    cxx_lambdas
)

target_include_directories(
      orca PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)

target_include_directories(
      orca PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/plog-1.1.4/include>
      $<INSTALL_INTERFACE:include>
  )

#################### Matplotlib #########################
if(PYTHONLIBS_FOUND AND NUMPY_FOUND)
    target_include_directories(
          orca PUBLIC
          $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
          $<BUILD_INTERFACE:${matplotlibcpp_include_dirs}>
          $<INSTALL_INTERFACE:include>
      )
    target_link_libraries(orca PUBLIC ${PYTHON_LIBRARIES})
    install(DIRECTORY ${matplotlibcpp_include_dirs}/matplotlibcpp/ DESTINATION include/matplotlibcpp)
endif()

#################### Public linking #########################
target_link_libraries(orca PUBLIC Eigen3::Eigen)
if(XENOMAI_FOUND)
    target_compile_definitions(orca PRIVATE CONFIG_XENO_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
    target_include_directories(orca PRIVATE ${XENOMAI_INCLUDE_DIRS})
    target_link_libraries(orca PRIVATE ${XENOMAI_LIBRARIES})
endif()

#################### Private linking #########################
target_link_libraries(orca PRIVATE
    idyntree-high-level
    ${PROJECT_NAME}_priv_qpOASES
    osqpstatic
    ${PROJECT_NAME}_priv_tinyxml
)

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
#         set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in)
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

#################### Catkin devel support #########################
# In a catkin environnement, overwrite the generated config.cmake
# so we can export customs targets and include directories like eigen3
# Note that we need to copy eigen3 for ubuntu 14.04 that has eigen 3.0.x
# and does not have a FindEigen3.cmake

# catkin devel support
# if(catkin_FOUND)
#
#     add_custom_target(  ${PROJECT_NAME}-targets-copy ALL
#                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake
#                         COMMAND ${CMAKE_COMMAND} -E copy
#                         ${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake
#                         ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/orcaTargets.cmake
#                         COMMENT "Copying 'orcaTargets.cmake' to '${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake'"
#                      )
#     add_custom_target(  ${PROJECT_NAME}-eigen-copy ALL
#                         COMMAND ${CMAKE_COMMAND} -E copy
#                         ${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake
#                         ${CATKIN_DEVEL_PREFIX}/share/cmake/FindEigen3.cmake
#                         COMMENT "Copying 'FindEigen3.cmake' to '${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake'"
#                      )
#     add_custom_target(  ${PROJECT_NAME}-config-copy ALL
#                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
#                         COMMAND ${CMAKE_COMMAND} -E copy
#                         ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
#                         ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/orcaConfig.cmake
#                         COMMENT "Copying 'orcaConfig.cmake' to '${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake'"
#                      )
# endif()


#################### ORCA Install #########################
# Exporting private targets because of https://gitlab.kitware.com/cmake/cmake/issues/17357
install(TARGETS orca
        ${PROJECT_NAME}_priv_qpOASES
        ${PROJECT_NAME}_priv_tinyxml
    EXPORT orcaTargets
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hh"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE
)

install(DIRECTORY external/plog-1.1.4/include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(FILES
    ${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/cmake
)

# Plain cmake support
if(NOT catkin_FOUND)

    set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    configure_package_config_file(cmake/orcaConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
        INSTALL_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
        PATH_VARS INCLUDE_INSTALL_DIR
    )
    # Cmake Package
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/orcaConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    export(EXPORT orcaTargets
        FILE "${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake"
        NAMESPACE orca::
    )

    # This creates a package in ~/.cmake so we don't have to 'make install'
    # and extend the CMAKE_PREFIX_PATH. This is equivalent to installing in /usr or
    # /usr/local (standard path) but without sudo rights.
    # With this, any cmake project calling find_package(orca) will find orca::orca

    export(PACKAGE orca)


    install(EXPORT orcaTargets
        FILE orcaTargets.cmake
        NAMESPACE orca::
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/orcaConfigVersion.cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
    )
endif()
