.. image:: _static/orca-l.png
    :width: 40px
    :align: left

ORCA Control
============

ORCA is c++ whole-body controller meant to solve a set of tasks under constraints.

The problem is written as a ``quadratic problem`` :

.. math::

    \min_{x} \frac{1}{2}x^tHx + x^tg

    \text{subject to}

    lb \leq  x \leq ub

    lb_A \leq Ax \leq ub_A

* ``x`` the optimisation vector
* ``H`` the hessian matrix (:math:`size(x) \times size(x)`)
* ``g`` the gradient vector (:math:`size(x) \times 1`)
* ``A`` the constraint matrix (:math:`size(x) \times size(x)`)
* ``lb`` and ``ub`` the lower and upper bounds of ``x`` (:math:`size(x) \times 1`)
* ``lbA`` and ``ubA`` the lower and upper bounds of ``A`` (:math:`size(x) \times 1`)

Tasks are written as **weighted euclidian distance function** :

.. math::
    
    w_{task} . \lVert \mathbf{E}x + \mathbf{f} \rVert_{W_{norm}}
    
* ``x`` the optimisation vector, or **part** of the optimisation vector
* ``E`` the linear matrix of the affine function (:math:`size(x) \times size(x)`)
* ``f`` the origin vector (:math:`size(x) \times 1`)
* ``w task``  (scalar :math:`[0:1]`)
* ``W norm`` the origin vector (:math:`size(x) \times 1`)


Constraints are written as **double bounded linear function** :

.. math::
    
    lb_C \leq Cx \leq ub_C

* ``C`` the constraint matrix (:math:`size(x) \times size(x)`)
* ``lbC`` and ``ubC`` the lower and upper bounds of ``A`` (:math:`size(x) \times 1`)


.. math::

    \underset{n\times 1}{\mathrm{Y}} =  \underset{n\times p}{X} \times 
    \underset{p\times 1}{\theta} + \underset{n\times 1}{\varepsilon}

.. code-block:: c++

    auto robot = std::make_shared<RobotDynTree>();
    if(!robot->loadModelFromFile(urdf_url))
    {
        return -1;
    }
    robot->setBaseFrame("link_0");
    robot->setGravity(Eigen::Vector3d(0,0,-9.81));


.. image:: _static/isir.png
    :width: 100px
    :align: left

.. image:: _static/cnrs.png
    :width: 100px

.. image:: _static/upmc.png
    :width: 250px

.. toctree::
    :name: install
    :caption: Installation and Configuration
    :glob:
    :maxdepth: 2

    install/*

.. toctree::
    :name: optim
    :caption: Optim
    :glob:
    :maxdepth: 2

    optim/*

.. toctree::
    :name: tasks
    :caption: Tasks
    :glob:
    :maxdepth: 2

    tasks/*

.. toctree::
    :name: constraints
    :caption: Constraints
    :glob:
    :maxdepth: 2

    cstrs/*

.. toctree::
    :name: tools
    :caption: Tools
    :glob:
    :maxdepth: 2

    tools/*