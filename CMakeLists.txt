cmake_minimum_required(VERSION 3.1)
project(orca VERSION 2.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CMakePackageConfigHelpers)

#################### Xenomai ###########################
find_package(Xenomai QUIET)

if(XENOMAI_FOUND)
    message(STATUS "[ORCA] Building with Xenomai mutexes")
endif()

#################### Eigen3 ###########################
find_package(Eigen3 3.2.92 QUIET)

if(NOT Eigen3_FOUND)
    if(EIGEN3_INCLUDE_DIR)
        unset(EIGEN3_INCLUDE_DIR CACHE)
    endif()
    message(WARNING "Using internal Eigen3 (3.3.90), please consider installing it standalone.")
    set(BUILD_TESTING OFF CACHE BOOL "Disable eigen tests")
    add_subdirectory(external/eigen-3.3.90/eigen3)
    set(ENV{EIGEN3_ROOT} ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90)
    set(EIGEN3_ROOTDIR $ENV{EIGEN3_ROOT})
    set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen-3.3.90/eigen3)
endif()


#################### qpOASES ##########################
find_package(qpOASES QUIET)

if(NOT qpOASES_FOUND)
    message(WARNING "Using internal qpOASES (3.2.1), please consider installing it standalone.")
    set(qpOASES_INCLUDE_DIRS external/qpOASES-3.2.1/include )
    file(GLOB qpOASES_srcs ${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES-3.2.1/src/*.cpp)

    add_library(qpOASES STATIC ${qpOASES_srcs})
    set(qpOASES_LIBRARIES qpOASES)
    target_include_directories(qpOASES PRIVATE external/qpOASES-3.2.1/include)
    install(DIRECTORY external/qpOASES-3.2.1/include
            DESTINATION include
            PATTERN ".svn" EXCLUDE
    )
else()
    add_library(qpOASES UNKNOWN IMPORTED)
endif()

#################### iDynTree #########################
find_package(iDynTree QUIET)
#################### catkin #########################
find_package(catkin QUIET)

if(NOT iDynTree_FOUND)
    message(WARNING "Using internal iDynTree (0.8.1), please consider installing it standalone.")

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

    add_subdirectory(external/idyntree-0.8.1)

    foreach(l : ${iDynTree_LIBRARIES_ALL})
        if(TARGET ${l})
            list(APPEND iDynTree_LIBRARIES ${l})
        endif()
    endforeach()

    add_library(iDynTree::idyntree-high-level ALIAS idyntree-high-level)
endif()

#################### Catkin ###########################

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
    cxx_final
    cxx_lambdas
    cxx_nullptr
)

if(XENOMAI_FOUND)
    target_compile_definitions(orca PRIVATE CONFIG_XENO_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
    target_include_directories(orca PUBLIC ${XENOMAI_INCLUDE_DIRS})
    target_link_libraries(orca PUBLIC ${XENOMAI_LIBRARIES})
endif()


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
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/plog-1.1.3/include>
      $<INSTALL_INTERFACE:include>
  )

target_include_directories(
      orca PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/json-2.1.1/include>
      $<INSTALL_INTERFACE:include>
  )


target_include_directories(orca PUBLIC $<TARGET_PROPERTY:iDynTree::idyntree-high-level,INCLUDE_DIRECTORIES>)
target_link_libraries(orca PUBLIC iDynTree::idyntree-high-level)
target_link_libraries(orca PRIVATE ${qpOASES_LIBRARIES})

#################### ORCA Examples #########################
add_subdirectory(examples)

#################### Linter for atom #########################
if(Unix)
add_custom_target(  ${PROJECT_NAME}-atom-files ALL
                    DEPENDS ${PROJECT_NAME}
                    COMMAND ${CMAKE_COMMAND} -E copy
                    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                    ${PROJECT_SOURCE_DIR}
                    COMMENT "Copying 'compile_commands.json' to '${PROJECT_SOURCE_DIR}'"
                 )
endif()

#################### ORCA Install #########################
if(NOT qpOASES_FOUND)
    install(TARGETS orca qpOASES DESTINATION lib EXPORT orcaTargets)
else()
    install(TARGETS orca DESTINATION lib EXPORT orcaTargets)
endif()


install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hh"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE
)

install(DIRECTORY external/plog-1.1.3/include/plog DESTINATION include)

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
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
)
