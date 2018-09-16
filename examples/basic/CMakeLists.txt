add_executable(00-load_robot_model 00-load_robot_model.cc)
target_link_libraries(00-load_robot_model orca)
install(TARGETS 00-load_robot_model DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(01-simple_controller 01-simple_controller.cc)
target_link_libraries(01-simple_controller orca)
install(TARGETS 01-simple_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(02-simulating_results 02-simulating_results.cc)
target_link_libraries(02-simulating_results orca)
install(TARGETS 02-simulating_results DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(03-configure_task_from_yaml 03-configure_task_from_yaml.cc)
target_link_libraries(03-configure_task_from_yaml orca)
install(TARGETS 03-configure_task_from_yaml DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(04-configure_robot_from_yaml 04-configure_robot_from_yaml.cc)
target_link_libraries(04-configure_robot_from_yaml orca)
install(TARGETS 04-configure_robot_from_yaml DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)


add_executable(05-configure_from_yaml 05-configure_from_yaml.cc)
target_link_libraries(05-configure_from_yaml orca)
install(TARGETS 05-configure_from_yaml DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)
