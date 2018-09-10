.. _building_docs:

*********************************
Building the documentation
*********************************

The ORCA documentation is composed of two parts. The **user's manual** (what you are currently reading) and the **API Reference**. Since ORCA is written entirely in ``c++`` the API documentation is generated with Doxygen. The manual, on the otherhand, is generated with python Sphinx... because frankly it is prettier.

Obviously, you can always visit the url: insert_url_here

to read the documentation online, but you can also generate it locally easily thanks to the magical powers of python.

How to build
=============

First we need to install some dependencies for python and of course doxygen.

Python dependencies
---------------------

.. code-block:: bash

    pip3 install -U --user pip sphinx sphinx-autobuild recommonmark sphinx_rtd_theme


or if using Python 2.x

.. code-block:: bash

    pip2 install -U --user pip sphinx sphinx-autobuild recommonmark sphinx_rtd_theme


Doxygen
------------

You can always install Doxygen from source by following:

.. code-block:: bash

    git clone https://github.com/doxygen/doxygen.git
    cd doxygen
    mkdir build
    cd build
    cmake -G "Unix Makefiles" ..
    make
    sudo make install

but we would recommend installing the binaries.

Linux:
^^^^^^^^^^^


.. code-block:: bash

    sudo apt install doxygen


OSX:
^^^^^^^^^^^


.. code-block:: bash

    brew install doxygen


Windows:
^^^^^^^^^^^

Download the executable file here: http://www.stack.nl/~dimitri/doxygen/download.html and follow the install wizard.


Building the docs with Sphinx
------------------------------

.. code-block:: bash

    cd [orca_root]
    cd docs/
    make html

``[orca_root]`` is the path to wherever you cloned the repo i.e. ``/home/$USER/orca/``.


How to browse
--------------

Since Sphinx builds static websites you can simply find the file ``docs/build/html/index.html`` and open it in a browser.

If you prefer to be a fancy-pants then you can launch a local web server by navigating to ``docs/`` and running:

.. code-block:: bash

    make livehtml

This method has the advantage of automatically refreshing when you make changes to the ``.rst`` files. You can browse the site at: http://127.0.0.1:8000.
