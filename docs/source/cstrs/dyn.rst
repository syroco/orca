Dynamics Equation Constraint
============================

* Control variable : X (whole optimisation vector)
* Size of the constraint : :math:`ndof \times size(X)`

.. math::
    
    w_{task} . \lVert \mathbf{E}x + \mathbf{f} \rVert_{W_{norm}}

.. code-block:: c++

    auto robot = std::make_shared<RobotDynTree>();
    if(!robot->loadModelFromFile(urdf_url))
    {
        return -1;
    }
    robot->setBaseFrame("link_0");
    robot->setGravity(Eigen::Vector3d(0,0,-9.81));