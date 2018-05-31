.. _install:

*************************************
Installation and Configuration
*************************************

This guide will take you through the steps to install ORCA on your machine. ORCA is cross platform so you should be able to install it on Linux, OSX, and Windows.

Dependencies
===============

* A modern **c++11** compiler (gcc > 4.8 or clang > 3.8)
* **cmake** > 3.1
* **iDynTree** (optional, shipped)
* **qpOASES** 3 (optional, shipped)
* **Eigen** 3 (optional, shipped)
* **Gazebo** 8 (optional)

ORCA is self contained! That means that is ships with both **iDynTree** and **qpOASES** inside the project, allowing for fast installations and easy integration on other platforms. Therefore you can start by simply building ORCA from source and it will include the necessary dependencies so you can get up and running.

Always keep in mind that it's better to install the dependencies separately if you plan to use **iDynTree** or **qpOASES** in other projects. For now only **iDynTree** headers appear in public headers, but will be removed eventually to ease the distribution of this library.

If you want to install the dependencies separately please read the following section: :ref:`install_deps`. Otherwise, if you just want to get coding, then jump ahead to :ref:`install_orca`.


.. note:: You can almost always avoid calling sudo, by calling ``cmake .. -DCMAKE_INSTALL_PREFIX=/some/dir`` and exporting the ``CMAKE_PREFIX_PATH`` variable : ``export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/some/dir``.


.. _install_deps:

Installing the dependencies
---------------------------

This installation requires you to build the dependencies separately, but will give you better control over versioning and getting the latest features and bug fixes.

Eigen
^^^^^^^^^^^^^^^

.. code-block:: bash
    :linenos:

    wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
    tar xjvf 3.3.4.tar.bz2
    cd eigen-eigen-dc6cfdf9bcec
    mkdir build ; cd build
    cmake --build .
    sudo cmake --build . --target install


qpOASES
^^^^^^^^^^^^^^^

.. code-block:: bash
    :linenos:

    wget https://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.1.zip
    unzip qpOASES-3.2.1.zip
    cd qpOASES-3.2.1
    mkdir build ; cd build
    cmake .. -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_BUILD_TYPE=Release
    cmake --build .
    sudo cmake --build . --target install


iDynTree
^^^^^^^^^^^^^^^

.. code-block:: bash
    :linenos:

    git clone https://github.com/robotology/idyntree
    cd idyntree
    mkdir build ; cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build .
    sudo cmake --build . --target install


Gazebo
^^^^^^^^^^^^^^^

Examples are built with Gazebo 8. They can be adapted of course to be backwards compatible.

.. code-block:: bash
    :linenos:

    curl -ssL http://get.gazebosim.org | sh




.. _install_orca:

Installing ORCA
==========================

Whether or not you have installed the dependencies separately, you are now ready to clone, build and install ORCA. Hooray.

.. code-block:: bash
    :linenos:

    git clone https://github.com/syroco/orca
    cd orca
    mkdir build ; cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build .
    sudo cmake --build . --target install


.. _test-install:

Testing your installation
================================

Assuming you followed the directions to the letter and encountered no compiler errors along the way, then you are ready to get started with ORCA. Before moving on to the :ref:`tutorials`, let's first test the installation.

To do so simply run the following command:

.. code-block:: bash
    :linenos:

    orca_install_test



What's next?
==================

Check out :ref:`where_to_go` for more info.
