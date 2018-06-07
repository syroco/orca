.. _overview:

Overview
============


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


Dynamics
===========

.. toctree::
    :maxdepth: 2

    dynamics/dynamics


Optimization
=================

.. toctree::
    :maxdepth: 2

    optim/optimization_vector
    optim/optimization_problem


Tasks
=============

.. toctree::
    :maxdepth: 2

    tasks/tasks
    tasks/cartesian_acceleration_task
    tasks/joint_acceleration_task
    tasks/wrench_task
    tasks/torque_task
    tasks/task_servoing

Constraints
===============

.. toctree::
    :maxdepth: 2

    constraints/constraints
    constraints/dynamics_equation
