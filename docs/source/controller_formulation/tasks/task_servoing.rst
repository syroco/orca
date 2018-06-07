
.. _task_servoing:

+++++++++++++++++++++++++
Task Servoing
+++++++++++++++++++++++++

The desired terms, :math:`\acc_{i}^{\text{des}}`, :math:`\jsrd_{i}^{\text{des}}`, :math:`\we_{i}^{\text{des}}`, and :math:`\torque^{\text{des}}`, from :eq:`acceleration_task`, :eq:`joint_space_acceleration_task_error`, :eq:`wrench_task`, and :eq:`torque_task`, respectively are provided by higher-level task servoing.
Commonly, the high-level reference of a task is simply to attain some pose, and in the case of a wrench task, some force and/or torque.
For acceleration tasks, if the desired task value is expressed as a pose, position, or orientation, then it must be converted to an acceleration. This is done here using a feedforward (PD) controller,



.. math::
    :label: feedforward_pd

    \acc_{i}^{\text{des}}(t+\Delta t) = \acc_{i}^{\text{ref}}(t+\Delta t) + K_{p}\bs{\epsilon}_{i}(t) + K_{d}\dot{\bs{\epsilon}}_{i}(t) \tc


\noindent where :math:`\acc_{i}^{\text{ref}}(t+\Delta t)` is the feedforward frame acceleration term, :math:`\bs{\epsilon}_{i}(t)` and  :math:`\dot{\bs{\epsilon}}_{i}(t)` are the current pose error and its derivative, with :math:`K_{p}` and :math:`K_{d}=2\sqrt{K_{p}}`, their proportional and derivative gains respectively.
This term also serves to remove drift at the controller level and stabilize the output of the task.
The terms, :math:`\bs{\epsilon}_{i}(t)` and  :math:`\dot{\bs{\epsilon}}_{i}(t)`, are not explicitly defined here because they are representation dependent (see \citep{Siciliano2008}).
For wrench and torque tasks a similar servoing controller can be developed using a Proportional-Integral (PI) controller.



.. math::
    :label: feedforward_pi

    \w^{des}(t+\Delta t) = \w^{ref}(t+\Delta t) + K_{p}\err_{\w}(t) + K_{i} \int \err_{\w}(t) dt


This servoing helps stabilize the whole-body controller by driving the desired task values to some fixed state in asymptotically stable manner.
Without the servoing the the task error objective term, :math:`\ft_{i}(\optvar)`, could change discontinuously between time steps resulting in discontinuous jumps in the optimal joint torques determined between two time steps.
