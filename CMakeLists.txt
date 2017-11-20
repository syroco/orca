cmake_minimum_required(VERSION 3.1)
project(orca)

set(CMAKE_POSITION_INDEPENDENT_CODE True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Use C++11
set(CMAKE_CXX_STANDARD 11)

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

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

#################### Catkin ###########################
find_package(catkin QUIET)
if(catkin_FOUND)
    catkin_package(INCLUDE_DIRS include)
endif()

#################### Catkin ###########################
include_directories(external/plog-1.1.3/include)

#################### JSON Parser ###########################
include_directories(external/json-2.1.1/include)


#################### qpOASES ##########################
find_package(qpOASES QUIET)

if(NOT qpOASES_FOUND)
    message(WARNING "Using internal qpOASES (3.2), please consider installing it standalone.")
    add_subdirectory(external/qpOASES-3.2)
    set(qpOASES_INCLUDE_DIRS external/qpOASES-3.2/include )
    set(qpOASES_LIBRARIES qpOASES)
endif()

include_directories(${qpOASES_INCLUDE_DIRS})

#################### iDynTree #########################
find_package(iDynTree QUIET)

if(NOT iDynTree_FOUND)
    set(iDynTree_DIR external/idyntree-0.8.1)
    add_subdirectory(${iDynTree_DIR})
    set(iDynTree_INCLUDE_DIRS   ${iDynTree_DIR}/src/core/include
                                ${iDynTree_DIR}/src/model/include
                                ${iDynTree_DIR}/src/sensors/include
                                ${iDynTree_DIR}/src/model_io/urdf/include
                                ${iDynTree_DIR}/src/estimation/include
                                ${iDynTree_DIR}/src/high-level/include
    )
    set(iDynTree_LIBRARIES idyntree-core
                           idyntree-model
                           idyntree-modelio-urdf
                           idyntree-regressors
                           idyntree-high-level)
    message(WARNING "Using internal iDynTree (0.8.1), please consider installing it standalone.")

endif()

include_directories(${iDynTree_INCLUDE_DIRS})


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
target_include_directories(orca PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(orca ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES})


#################### ORCA Examples #########################
add_subdirectory(examples)

#################### OROCOS Wrapper ########################
find_package(OROCOS-RTT QUIET ${RTT_HINTS})
if(FALSE OR OROCOS-RTT_FOUND)
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

    add_definitions(-DUSE_OROCOS_MUTEXES)
    include_directories(${OROCOS-RTT_INCLUDE_DIRS})
    target_link_libraries(orca ${OROCOS-RTT_LIBRARIES})

    orocos_use_package(ocl-taskbrowser)
    orocos_use_package(ocl-deployment)
    include_directories(${OROCOS-RTT_INCLUDE_DIRS})
    orocos_component(rtt-contact rtt-src/rtt-Contact.cc)
    orocos_component(rtt-dyneq rtt-src/rtt-DynEq.cc)
    orocos_component(rtt-reset-test rtt-src/rtt-reset-test.cc)
    orocos_plugin(rtt-signals-tests rtt-src/rtt-signals-tests.cc)
    target_link_libraries(rtt-contact orca)
    target_link_libraries(rtt-dyneq orca)
    orocos_generate_package()
endif()


if(Unix)
add_custom_target(  orca-atom-files ALL
                    DEPENDS orca
                    COMMAND ${CMAKE_COMMAND} -E copy
                    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
                    ${PROJECT_SOURCE_DIR}
                    COMMENT "Copying 'compile_commands.json' to '${PROJECT_SOURCE_DIR}'"
                 )
endif()
