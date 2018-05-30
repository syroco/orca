.. _overview:

**************************************
Overview
**************************************





The problem is written as a **quadratic problem** :

.. math::

    \min_{x} \frac{1}{2}x^tHx + x^tg

    \text{subject to}

    lb \leq  x \leq ub

    lb_A \leq Ax \leq ub_A

* ``x`` the optimization vector
* ``H`` the hessian matrix (:math:`size(x) \times size(x)`)
* ``g`` the gradient vector (:math:`size(x) \times 1`)
* ``A`` the constraint matrix (:math:`size(x) \times size(x)`)
* ``lb`` and ``ub`` the lower and upper bounds of ``x`` (:math:`size(x) \times 1`)
* ``lbA`` and ``ubA`` the lower and upper bounds of ``A`` (:math:`size(x) \times 1`)

Tasks are written as **weighted euclidian distance function** :

.. math::

    w_{task}  \lVert \mathbf{E}x + \mathbf{f} \rVert_{W_{norm}}^2

* ``x`` the optimization vector, or **part** of the optimization vector
* ``E`` the linear matrix of the affine function (:math:`size(x) \times size(x)`)
* ``f`` the origin vector (:math:`size(x) \times 1`)
* ``w task`` the weight of the tasks in the overall quadratic cost (scalar :math:`[0:1]`)
* ``W norm`` the weight of the euclidean norm (:math:`size(x) \times size(x)`)

Given n_t tasks, the **overall cost function** is such that:

.. math::

    \frac{1}{2}x^tHx + x^tg = \frac{1}{2} \sum_{i=1}^{n_t}  w_{task,i}  \lVert \mathbf{E}_ix + \mathbf{f}_i \rVert_{W_{norm,i}}^2

Constraints are written as **double bounded linear function** :

.. math::

    lb_C \leq Cx \leq ub_C

* ``C`` the constraint matrix (:math:`size(x) \times size(x)`)
* ``lbC`` and ``ubC`` the lower and upper bounds of ``A`` (:math:`size(x) \times 1`)


.. .. math::

..    \underset{n\times 1}{\mathrm{Y}} =  \underset{n\times p}{X} \times
..    \underset{p\times 1}{\theta} + \underset{n\times 1}{\varepsilon}

.. .. code-block:: c++
..
..   auto robot = std::make_shared<RobotDynTree>();
..    if(!robot->loadModelFromFile(urdf_url))
..    {
..        return -1;
..    }
..    robot->setBaseFrame("link_0");
..    robot->setGravity(Eigen::Vector3d(0,0,-9.81));
