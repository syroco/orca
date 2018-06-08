.. _dynamics:

***********************
Dynamics Constraints
***********************



The rigid body dynamics of the robot are governed by the equations of motion from :ref:`equations_of_motion_in_optvar`.
This constraint ultimately dictates the achievable dynamics of the system, and is formulated as the following equality constraint,


.. math::
    :label: equations_of_motion_in_optvar_A_and_b

    \underbrace{\bmat{-M(\q) & S^{\top} & \Je^{\top}(\q)}}_{A^{d}}\optvar = \underbrace{\bs{n}(\q, \jsr)}_{\bs{b}^{d}} \tp


The terms :math:`A^{d}` and :math:`\bs{b}^{d}` are used to distinguish the equality constraint matrix and vector, respectively, for the dynamic constraints.


.. important::

    To put this into ORCA standard form we have,

    .. math::

        \bs{b}^{d} \leq A^{d}\optvar \leq \bs{b}^{d}
