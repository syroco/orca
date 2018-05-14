CmakeList for using Orca and Traxxs
--------------------------------------------------

.. code-block:: bash

  cmake_minimum_required(VERSION 3.1.0)
  project("your_project")

  ########### orocos ############
  find_package(orocos_kdl REQUIRED)
  ########### orca  #############
  find_package(orca REQUIRED)
  find_package(gazebo REQUIRED)
  find_package(traxxs REQUIRED)
  link_directories(${GAZEBO_LIBRARY_DIRS})


  include_directories(
      ${orocos_kdl_INCLUDE_DIRS}
      ${GAZEBO_INCLUDE_DIRS}
  )


  # Warning pour gazebo 8
  list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -fext-numeric-literals")

  add_executable(demo0 demo0.cpp)
  target_link_libraries(demo0 ${orocos_kdl_LIBRARIES} orca::orca ${GAZEBO_LIBRARIES} pthread tinyxml traxxs::traxxs traxxs::traxxs_softmotion)
  install(TARGETS demo0 DESTINATION devel/lib)