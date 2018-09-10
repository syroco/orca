.. _using_the_lib:

++++++++++++++++++++++++++++++
Using ORCA in your projects
++++++++++++++++++++++++++++++

If you want to you ORCA in your project you can either use pure ``CMake`` or ``catkin``.


CMake
===========================


.. code-block:: cmake

    # You need at least version 3.1 to use the modern CMake targets.
    cmake_minimum_required(VERSION 3.1.0)

    # Your project's name
    project(my_super_orca_project)

    # Tell CMake to find ORCA
    find_package(orca REQUIRED)

    # Add your executable(s) and/or library(ies) and their corresponding source files.
    add_executable(${PROJECT_NAME} my_super_orca_project.cc)

    # Point CMake to the ORCA targets.
    target_link_libraries(${PROJECT_NAME} orca::orca)





catkin
===========================

.. note:: As of now, ``catkin`` does not support modern cmake targets and so you have some superfluous cmake steps to do when working with ``catkin`` workspaces.

.. code-block:: cmake

    # You need at least version 2.8.3 to use the modern CMake targets.
    cmake_minimum_required(VERSION 2.8.3)

    # Your project's name
    project(my_super_orca_catkin_project)

    # Tell CMake to find ORCA
    find_package(orca REQUIRED)

    # Tell catkin to find ORCA
    find_package(catkin REQUIRED COMPONENTS orca)

    # Include the catkin headers
    include_directories(${catkin_INCLUDE_DIRS})

    # Add your executable(s) and/or library(ies) and their corresponding source files.
    add_executable(${PROJECT_NAME} my_super_orca_catkin_project.cc)

    # Point CMake to the catkin and ORCA targets.
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} orca::orca)
