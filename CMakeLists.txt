cmake_minimum_required(VERSION 3.1)
project(orca VERSION 0.1.0)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
    message(WARNING "Using internal Eigen3 (3.3.4), please consider installing it standalone.")
    # This is for iDynTree to find the library
    set(ENV{EIGEN3_ROOT} ${PROJECT_SOURCE_DIR}/external/eigen-3.3.4)
    set(EIGEN3_ROOTDIR $ENV{EIGEN3_ROOT})
    set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen-3.3.4/eigen3)
endif()


#################### qpOASES ##########################
find_package(qpOASES QUIET)

if(NOT qpOASES_FOUND)
    message(WARNING "Using internal qpOASES (3.2), please consider installing it standalone.")
    set(qpOASES_INCLUDE_DIRS external/qpOASES-3.2/include )
    set(qpOASES_LIBRARIES qpOASES)
endif()

#################### iDynTree #########################
find_package(iDynTree QUIET)

if(NOT iDynTree_FOUND)
    set(iDynTree_DIR external/idyntree-0.8.1)
    set(PACKAGE_iDynTree_INCLUDEDIR "${CMAKE_INSTALL_PREFIX}")
    message(WARNING "Using internal iDynTree (0.8.1), please consider installing it standalone.")
endif()

#################### Catkin ###########################
find_package(catkin QUIET)
if(catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${iDynTree_INCLUDE_DIRS} external/plog-1.1.3/include external/json-2.1.1/include
        LIBRARIES orca idyntree-high-level idyntree-core idyntree-model idyntree-modelio-urdf idyntree-sensors tinyxml
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
target_compile_features(${PROJECT_NAME} PUBLIC 
    cxx_auto_type
    cxx_final
    cxx_lambdas
    cxx_nullptr
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)

if(XENOMAI_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_XENO_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
    target_include_directories(${PROJECT_NAME} PUBLIC ${XENOMAI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${XENOMAI_LIBRARIES})
endif()


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
else()
    target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${qpOASES_INCLUDE_DIRS})

target_include_directories(
      ${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/plog-1.1.3/include>
      $<INSTALL_INTERFACE:include>
  )

target_include_directories(
      ${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/json-2.1.1/include>
      $<INSTALL_INTERFACE:include>
  )


target_include_directories(${PROJECT_NAME} PUBLIC $<TARGET_PROPERTY:iDynTree::idyntree-high-level,INCLUDE_DIRECTORIES>)
target_link_libraries(${PROJECT_NAME} PUBLIC iDynTree::idyntree-high-level)
target_link_libraries(${PROJECT_NAME} PRIVATE ${qpOASES_LIBRARIES})

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
install(TARGETS ${PROJECT_NAME} DESTINATION lib EXPORT orcaTargets)
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

if(catkin_FOUND)
    add_custom_target(  ${PROJECT_NAME}-targets-copy ALL
                        DEPENDS ${PROJECT_NAME}
                        COMMAND ${CMAKE_COMMAND} -E copy
                        ${CMAKE_CURRENT_BINARY_DIR}/orcaTargets.cmake
                        ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake
                        COMMENT "Copying 'orcaTargets.cmake' to '${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake'"
                     )

    file(COPY ${CMAKE_CURRENT_BINARY_DIR}/orcaConfigVersion.cmake DESTINATION ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake)
    file(COPY ${CMAKE_CURRENT_BINARY_DIR}/orcaConfig.cmake DESTINATION ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake)
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
