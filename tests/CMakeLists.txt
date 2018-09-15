# add_executable(orca_install_test orca_install_test.cc)
# target_link_libraries(orca_install_test orca)
# install(TARGETS orca_install_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(orca_parser_test orca_parser_test.cc)
target_link_libraries(orca_parser_test orca)

