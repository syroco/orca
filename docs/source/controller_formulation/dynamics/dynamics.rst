.. _dynamics:

Free-Floating Rigid Body Dynamics
========================================

For robots whose root link can float freely in Cartesian space, e.g. humanoids, it is necessary to consider the pose of the root link with respect to (\wrt) the inertial reference frame. The primary method for doing so is to account for the root link pose directly in the generalized coordinates, :math:`\q`, of the robot as shown by.

.. todo::

    add citations


.. \citep{Sentis2005,Mistry2010,Righetti2011}.
..
.. \begin{wrapfigure}{r}{0.4\textwidth}
.. \centering
.. \includegraphics[width=\linewidth]{/whole_body_control/floating_base_robot}
.. \caption{A diagram indicating visually what it means to include the root link pose in the parameterization. The 6-\dof\ of the floating base are modeled as a 6-\dof\ linkage with the world or inertial frame. Image taken from \citep{Mistry2010}.}
.. \end{wrapfigure}
..


The generalized configuration parameterization for floating base robots,


.. math::
    :label: qDef

    \q = \begin{Bmatrix}
    \gtc_{fb} \\ \q_{j}
    \end{Bmatrix} \tc




therefore contains the pose of the base link \wrt\ the inertial reference frame, :math:`\gtc_{fb}`, and the joint space coordinates, :math:`\q_j`. Set brackets are used in :eq:`qDef` because :math:`\gtc_{fb}` is a homogeneous transformation matrix in :math:`\R^{4\times4}` and :math:`\q_j` is a vector in :math:`\R^{n}`, with :math:`n` the number of \dof\ of the robot, thus :math:`\gtc_{fb}` and :math:`\q_{j}` cannot be concatenated into a vector.
However, the twist of the base, :math:`\bs{v}_{fb}`, with the joint velocities, :math:`\qd_{j}`, can be concatenated in vector notation, along with the base and joint accelerations to obtain,



.. math::
    :label: jsrDef

    \jsr = \begin{bmatrix}
    \bs{v}_{fb} \\ \qd_{j}
    \end{bmatrix}
    \tc
    \quad
    \text{and}
    \quad
    \jsrd = \begin{bmatrix}
    \dot{\bs{v}}_{fb} \\ \qdd_{j}
    \end{bmatrix} \tp





These representations provide a complete description of the robot's state and its rate of change, and allow the equations of motion to be written as,




.. math::
    :label: big_equation_of_motion

    M(\q)\jsrd + \underbrace{C(\q, \jsr)\jsr + \bs{g}(\q)}_{\bs{n}(\q, \jsr)} = S^{\top}\torque + \Je^{\top}(\q)\we \tp






In :eq:`big_equation_of_motion`, :math:`M(\q)` is the generalized mass matrix, :math:`C(\q, \jsr)\jsr` and :math:`\bs{g}(\q)` are the Coriolis-centrifugal and gravitational terms, :math:`S` is a selection matrix indicating the actuated degrees of freedom, :math:`\we` is the concatenation of the external contact wrenches, and :math:`\Je` their concatenated Jacobians.


Grouping :math:`C(\q, \jsr)\jsr` and :math:`\bs{g}(\q)` together into :math:`\bs{n}(\q, \jsr)`, the equations can by simplified to

.. math::
    :label: equation_of_motion

    M(\q)\jsrd + \bs{n}(\q, \jsr) = S^{\top}\torque + \Je^{\top}(\q)\we \tp


The joint torques induced by friction force could also be included in :eq:`equation_of_motion`, but are left out for the sake of simplicity.
Additionally, the variables :math:`\jsrd`, :math:`\tau`, and :math:`\we`, can be grouped into the same vector,

.. math::
    :label: optvar

    \optvar = \bmat{\jsrd \\ \torque \\ \we} \tc



forming the optimization variable from :eq:`generic_whole_body_controller`, and allowing :eq:`equation_of_motion` to be rewritten as,

.. math::
    :label: equations_of_motion_in_optvar

    \bmat{-M(\q) & S^{\top} & \Je^{\top}(\q)}\optvar = \bs{n}(\q, \jsr) \tp


Equation :eq:`equations_of_motion_in_optvar` provides an equality constraint which can be used to ensure that the minimization of the control objectives respects the system dynamics.
