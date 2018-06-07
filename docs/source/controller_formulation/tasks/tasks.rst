
.. _tasks:

******************************
Control Objectives (Tasks)
******************************

The basic problem of control is to drive a system from some initial state to some desired state. The control of robots is no different, but the term state takes on greater ambiguity.
For simple systems, such as the double integrator, linearized inverted pendulum, etc., state-space control is sufficient for virtually any high-level objective one could envision for the system.
However, for a robot, describing the control problem solely in terms of its state, i.e. :math:`\q` and :math:`\jsr`, is limiting and one may also want to describe it in terms of the pose and twist of an end-effector, or possibly even a wrench on some link (although not technically a state in the classical control sense).
Far from being a detriment, this variability is what makes robots so useful but requires a bit of abstraction from classical state-space control vocabulary. For this reason, the term **task** is commonly used to indicate a control objective for a robot. Tasks, in second-order controllers, can be driven by desired accelerations, wrenches, or torques, and in operational-space or joint-space.
They are expressed in the whole-body controller as functions of the errors between the desired and current values of the task. In this work, the square of the :math:`l^{2}`-norm is used to create a quadratic objective function.
Consequently, the task errors are expressed in the least-squares formulation.
