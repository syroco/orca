.. _quick_start:


*********************************
Quick Start Guide
*********************************

First off, make sure you have followed the :ref:`install` guide step by step.


If you have successfully installed ORCA then we can go ahead and try out one of the examples to get things up and running. To do so we will launch the example: ``06-trajectory_following`` (more info here: :ref:`06-trajectory_following`)

This example assumes you have Gazebo >=8.0 installed on your machine. If not please follow the Gazebo tutorial for your system (http://gazebosim.org/tutorials?cat=install) and rebuild the ORCA library.

Once you have Gazebo, to launch the example open a terminal and run:

.. code-block:: bash

    06-trajectory_following [path_to_orca]/examples/resources/lwr.urdf



.. important::

    Make sure to replace ``[path_to_orca]`` with the real path to the ORCA repo on your system.

Now, open a second terminal and run:

.. code-block:: bash

    gzclient


If everything goes well then you should see the robot moving back and forth like this:


.. image:: ../../_static/orca_test.gif
    :width: 600px
    :align: center





What's next?
==================

Check out :ref:`where_to_go` for more info.
