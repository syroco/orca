add_executable(orca-demo0 orca-demo0.cc)
target_link_libraries(orca-demo0 orca)

# add_executable(orca-demo1 orca-demo1.cc)
# target_link_libraries(orca-demo1 orca)

add_executable(orca-idyntree-example iDynTreeExample.cc)
target_link_libraries(orca-idyntree-example orca)

add_executable(orca-idyntree-tests iDynTreeTests.cc)
target_link_libraries(orca-idyntree-tests orca)

find_package(gazebo QUIET)

if(gazebo_FOUND)
    if(${GAZEBO_VERSION} VERSION_GREATER "7.0" AND ${GAZEBO_VERSION} VERSION_LESS "9.0")
        message(STATUS "Gazebo ${GAZEBO_VERSION} found, building examples")

        include_directories(${GAZEBO_INCLUDE_DIRS})
        link_directories(${GAZEBO_LIBRARY_DIRS})
        list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -fext-numeric-literals")

        add_executable(orca-demo1-gazebo orca-demo1-gazebo.cc)
        target_link_libraries(orca-demo1-gazebo ${GAZEBO_LIBRARIES} pthread orca tinyxml)
        install(TARGETS orca-demo1-gazebo DESTINATION lib/orca)
    else()
        message(WARNING "Gazebo ${GAZEBO_VERSION} found, we need at least gazebo 7 or 8")
    endif()
else()
    message(STATUS "Gazebo not found, not building gazebo examples")
endif()

# install(TARGETS orca-demo1 DESTINATION lib/orca)
