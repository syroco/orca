.. _overview:

**************************************
Overview
**************************************


The most generic form of the whole-body controller can be summarized by the following optimization problem,


.. math::
    :label: generic_whole_body_controller

    \underset{\optvar}{\argmin} &\quad \ft(\optvar)   \\
    \text{s.t.}            &\quad G\optvar \leq \bs{h}  \\
                           &\quad A\optvar = \bs{b}  \tp


* s.t.: subject to


The objective, :math:`\ft(\optvar)`, is a function of the optimization variable, :math:`\optvar`, and is determined by control objectives, or tasks.
The resolution of the objective is subject to (s.t.) the inequality and equality constraints, which ensure that the control constraints are respected.
To understand how whole-body controllers are formulated in ORCA, we begin with a brief description of the free-floating rigid body dynamics. The parameterization of the dynamics forms the optimization variable. The control objectives, or tasks, and constraints are then detailed and written in terms of the optimization variable. Finally, task prioritization schemes are discussed.







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
    \gtc_{b} \\ \q_{j}
    \end{Bmatrix} \tc




therefore contains the pose of the base link \wrt\ the inertial reference frame, :math:`\gtc_b`, and the joint space coordinates, :math:`\q_j`. Set brackets are used in :eq:`qDef` because :math:`\gtc_b` is a homogeneous transformation matrix in :math:`\R^{4\times4}` and :math:`\q_j` is a vector in :math:`\R^{n}`, with :math:`n` the number of \dof\ of the robot, thus :math:`\gtc_{b}` and :math:`\q_{j}` cannot be concatenated into a vector.
However, the twist of the base, :math:`\bs{v}_{b}`, with the joint velocities, :math:`\qd_{j}`, can be concatenated in vector notation, along with the base and joint accelerations to obtain,



.. math::
    :label: jsrDef

    \jsr = \begin{bmatrix}
    \bs{v}_{b} \\ \qd_{j}
    \end{bmatrix}
    \tc
    \quad
    \text{and}
    \quad
    \jsrd = \begin{bmatrix}
    \dot{\bs{v}}_{b} \\ \qdd_{j}
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








How things are implemented
============================


The problem is written as a **quadratic problem** :

.. math::

    \min_{x} \frac{1}{2}x^tHx + x^tg

    \text{subject to}

    lb \leq  x \leq ub

    lb_A \leq Ax \leq ub_A

* :math:`x` the optimization vector
* :math:`H` the hessian matrix (:math:`size(x) \times size(x)`)
* :math:`g` the gradient vector (:math:`size(x) \times 1`)
* :math:`A` the constraint matrix (:math:`size(x) \times size(x)`)
* :math:`lb` and ``ub`` the lower and upper bounds of ``x`` (:math:`size(x) \times 1`)
* :math:`lbA` and ``ubA`` the lower and upper bounds of ``A`` (:math:`size(x) \times 1`)

Tasks are written as **weighted euclidian distance function** :

.. math::

    w_{task}  \lVert \mathbf{E}x + \mathbf{f} \rVert_{W_{norm}}^2

* :math:`x` the optimization vector, or **part** of the optimization vector
* :math:`E` the linear matrix of the affine function (:math:`size(x) \times size(x)`)
* :math:`f` the origin vector (:math:`size(x) \times 1`)
* :math:`w task` the weight of the tasks in the overall quadratic cost (scalar :math:`[0:1]`)
* :math:`W norm` the weight of the euclidean norm (:math:`size(x) \times size(x)`)

Given n_t tasks, the **overall cost function** is such that:

.. math::

    \frac{1}{2}x^tHx + x^tg = \frac{1}{2} \sum_{i=1}^{n_t}  w_{task,i}  \lVert \mathbf{E}_ix + \mathbf{f}_i \rVert_{W_{norm,i}}^2

Constraints are written as **double bounded linear function** :

.. math::

    lb_C \leq Cx \leq ub_C

* :math:`C` the constraint matrix (:math:`size(x) \times size(x)`)
* :math:`lbC` and :math:`ubC` the lower and upper bounds of :math`A` (:math:`size(x) \times 1`)
