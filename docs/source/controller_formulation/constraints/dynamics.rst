.. _dynamics:

***********************
Dynamics
***********************



The rigid body dynamics of the robot are governed by the equations of motion from :ref:`equations_of_motion_in_optvar`.
This constraint ultimately dictates the achievable dynamics of the system, and is formulated as the following equality constraint,


.. math::
    :label: equations_of_motion_in_optvar_A_and_b

    \underbrace{\bmat{-M(\q) & S^{\top} & \Je^{\top}(\q)}}_{A^{d}}\optvar = \underbrace{\bs{n}(\q, \jsr)}_{\bs{b}^{d}} \tp


The terms :math:`A^{d}` and :math:`\bs{b}^{d}` are used to distinguish the equality constraint matrix and vector, respectively, for the dynamic constraints.
This is done because the constraints presented here are combined by vertically concatenating them to form :ref:`generic_whole_body_controller_equality` and :ref:`generic_whole_body_controller_inequality` in the controller.


To refactor
==============

.. todo::

    refactor this section

* **Control variable** : X (whole optimization vector)
* **Type** : Equality constraint
* **Size** : :math:`ndof \times size(X)`

.. math::

    \begin{bmatrix}
    - M &&
    S_{\tau} &&
    J_{^{e}w}
    \end{bmatrix} X
    = C + G

.. code-block:: c++

    orca::constraint::DynamicsEquationConstraint dyn_eq;
    dyn_eq.loadRobotModel( urdf );
    dyn_eq.setGravity( Eigen::Vector3d(0,0,-9.81) );
    dyn_eq.update(); // <-- Now initialized

    dyn_eq.activate(); // <-- Now activated
    dyn_eq.insertInProblem(); // <-- Now part of the optimization problem
