set(CMAKE_CXX_STANDARD 11)

add_executable(orca-demo1 orca-demo1.cc)
target_link_libraries(orca-demo1 orca)

add_executable(orca-idyntree-example iDynTreeExample.cc)
target_link_libraries(orca-idyntree-example orca)

add_executable(orca-idyntree-tests iDynTreeTests.cc)
target_link_libraries(orca-idyntree-tests orca)


find_package(gazebo 8 QUIET)

if(gazebo_FOUND)
    message(STATUS "Gazebo ${GAZEBO_VERSION} found, building examples")

    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

    add_executable(orca-demo1-gazebo orca-demo1-gazebo.cc)
    target_link_libraries(orca-demo1-gazebo ${GAZEBO_LIBRARIES} pthread orca tinyxml)

endif()
