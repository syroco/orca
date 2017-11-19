.. image:: _static/orca-l.png
    :width: 50px
    :align: left

ORCA is an optimisation-based library to solve robotics problems with tasks and constraints.

.. math::

    \min_{x} \frac{1}{2}x^tHx + x^tg

    \text{subject to}

    lb \leq  x \leq ub

    lb_A \leq Ax \leq ub_A


.. code-block:: c++

    auto robot = std::make_shared<RobotDynTree>();
    if(!robot->loadModelFromFile(urdf_url))
    {
        return -1;
    }
    robot->setBaseFrame("link_0");
    robot->setGravity(Eigen::Vector3d(0,0,-9.81));


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

.. image:: _static/isir.png
    :width: 100px
    :align: left

.. image:: _static/cnrs.png
    :width: 100px

.. image:: _static/upmc.png
    :width: 250px

