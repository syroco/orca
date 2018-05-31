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


.. code-block:: bash

    poop


Building the docs with Sphinx
------------------------------

.. code-block:: bash

    cd docs/
    make html



How to browse
--------------

Since Sphinx builds static websites you can simply find the file ``docs/build/html/index.html`` and open it in a browser.

If you prefer to be a fancy-pants then you can launch a local web server by navigating to ``docs/`` and running:

.. code-block:: bash

    make livehtml

You can browse the site at:
