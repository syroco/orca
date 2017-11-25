cmake_minimum_required(VERSION 3.1)
project(orca VERSION 0.1.0)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#################### Eigen3 ###########################
find_package(Eigen3 3.2.92 QUIET)

if(NOT Eigen3_FOUND)
    if(EIGEN3_INCLUDE_DIR)
        unset(EIGEN3_INCLUDE_DIR CACHE)
    endif()
    message(WARNING "Using internal Eigen3 (3.3.4), please consider installing it standalone.")
    set(ENV{EIGEN3_ROOT} ${PROJECT_SOURCE_DIR}/external/eigen-3.3.4)
    find_package(Eigen3 3.2.92 REQUIRED)
endif()

include_directories(${EIGEN3_INCLUDE_DIR})


#################### PLOG ###########################
include_directories(external/plog-1.1.3/include)

#################### JSON Parser ###########################
include_directories(external/json-2.1.1/include)


#################### qpOASES ##########################
find_package(qpOASES QUIET)

if(NOT qpOASES_FOUND)
    message(WARNING "Using internal qpOASES (3.2), please consider installing it standalone.")
    set(qpOASES_INCLUDE_DIRS external/qpOASES-3.2/include )
    set(qpOASES_LIBRARIES qpOASES)
endif()

include_directories(${qpOASES_INCLUDE_DIRS})

#################### iDynTree #########################
find_package(iDynTree QUIET)

if(NOT iDynTree_FOUND)
    set(iDynTree_DIR external/idyntree-0.8.1)
    set(PACKAGE_iDynTree_INCLUDEDIR "${CMAKE_INSTALL_PREFIX}")
    message(WARNING "Using internal iDynTree (0.8.1), please consider installing it standalone.")
endif()

#include_directories(${iDynTree_INCLUDE_DIRS})

#################### Catkin ###########################
find_package(catkin QUIET)
if(catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${iDynTree_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS} external/plog-1.1.3/include external/json-2.1.1/include
        LIBRARIES orca ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES}
    )
endif()

# Add the subdirs to build the external libraries
# These have to be after catkin_package()
# so it can catch the targets

if(NOT iDynTree_FOUND)
    add_subdirectory(${iDynTree_DIR})
    add_library(iDynTree::idyntree-high-level ALIAS idyntree-high-level)
endif()

if(NOT qpOASES_FOUND)
    add_subdirectory(external/qpOASES-3.2 EXCLUDE_FROM_ALL)
endif()

#################### ORCA #########################
set(sub-dirs
    optim
    constraint
    common
    math
    robot
    task
    util
)

foreach(sub : ${sub-dirs})
    file(GLOB sub-srcs src/${sub}/*.cc)
    list(APPEND lib-srcs ${sub-srcs})
endforeach()

source_group("Source Files" FILES ${lib-srcs})

foreach(sub : ${sub-dirs})
    file(GLOB sub-hdrs include/orca/${sub}/*.h)
    list(APPEND lib-hdrs ${sub-hdrs})
endforeach()

source_group("Header Files" FILES ${lib-hdrs})

add_library(orca ${lib-srcs})
set_target_properties(orca PROPERTIES CXX_STANDARD 11)
set_target_properties(orca PROPERTIES CXX_STANDARD_REQUIRED ON)
target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

if(NOT Eigen3_FOUND)
	target_include_directories(
  		${PROJECT_NAME} PUBLIC
  		$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  		$<INSTALL_INTERFACE:include/eigen3>
  	)
endif()


target_include_directories(orca PUBLIC $<TARGET_PROPERTY:iDynTree::idyntree-high-level,INCLUDE_DIRECTORIES>)
target_link_libraries(orca PUBLIC iDynTree::idyntree-high-level)
target_link_libraries(orca PRIVATE ${qpOASES_LIBRARIES})

#################### ORCA Examples #########################
add_subdirectory(examples)

#################### Linter for atom #########################
if(Unix)
add_custom_target(  orca-atom-files ALL
                    DEPENDS orca
                    COMMAND ${CMAKE_COMMAND} -E copy
                    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                    ${PROJECT_SOURCE_DIR}
                    COMMENT "Copying 'compile_commands.json' to '${PROJECT_SOURCE_DIR}'"
                 )
endif()

#################### ORCA Install #########################
install(TARGETS orca DESTINATION lib EXPORT orcaTargets)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE)

if(NOT Eigen3_FOUND)
	install(DIRECTORY external/eigen-3.3.4/eigen3 DESTINATION include)
endif()

install(DIRECTORY external/plog-1.1.3/include/plog DESTINATION include)

#
# Cmake Package
#
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/orcaConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

set(CONFIG_PACKAGE_LOCATION lib/cmake/orca)
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
