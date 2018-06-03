add_executable(orca_install_test orca_install_test.cc)
target_link_libraries(orca_install_test orca Eigen3::Eigen)
install(TARGETS orca_install_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
