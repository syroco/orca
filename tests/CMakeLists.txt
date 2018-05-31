add_executable(orca_install_test orca_install_test.cc)
target_link_libraries(orca_install_test orca)
install(TARGETS orca_install_test DESTINATION bin/orca)
