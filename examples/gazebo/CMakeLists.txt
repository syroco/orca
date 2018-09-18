find_package(gazebo QUIET)

if(gazebo_FOUND)
    if(${GAZEBO_VERSION} VERSION_GREATER "7.0")
        message(STATUS "Gazebo ${GAZEBO_VERSION} found, building examples")

        include_directories(${GAZEBO_INCLUDE_DIRS})
        link_directories(${GAZEBO_LIBRARY_DIRS})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

        add_definitions(-DGAZEBO_VERSION_MAJOR=${gazebo_VERSION_MAJOR})

        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
            add_definitions(-fext-numeric-literals)
        endif()

        add_executable(00-load_gazebo_model 00-load_gazebo_model.cc)
        target_link_libraries(00-load_gazebo_model ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 00-load_gazebo_model DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(01-single_robot 01-single_robot.cc)
        target_link_libraries(01-single_robot ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 01-single_robot DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(02-multi_robot 02-multi_robot.cc)
        target_link_libraries(02-multi_robot ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 02-multi_robot DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(03-set_robot_state 03-set_robot_state.cc)
        target_link_libraries(03-set_robot_state ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 03-set_robot_state DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(04-set_robot_state_gravity_compensation 04-set_robot_state_gravity_compensation.cc)
        target_link_libraries(04-set_robot_state_gravity_compensation ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 04-set_robot_state_gravity_compensation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(05-orca_gazebo 05-orca_gazebo.cc)
        target_link_libraries(05-orca_gazebo ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 05-orca_gazebo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(06-trajectory_following 06-trajectory_following.cc)
        target_link_libraries(06-trajectory_following ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 06-trajectory_following DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

        add_executable(07-orca_gazebo_wrench_task 07-orca_gazebo_wrench_task.cc)
        target_link_libraries(07-orca_gazebo_wrench_task ${GAZEBO_LIBRARIES} pthread orca tinyxml Eigen3::Eigen)
        install(TARGETS 07-orca_gazebo_wrench_task DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)
    else()
        message(WARNING "Gazebo ${GAZEBO_VERSION} found, we need at least gazebo 7")
    endif()
else()
    message(STATUS "Gazebo not found, not building gazebo examples")
endif()
