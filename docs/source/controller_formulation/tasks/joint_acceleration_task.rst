
.. _joint_acc_task:

************************
Joint Acceleration Task
************************

Acceleration tasks can be expressed in either joint-space or in operational-space. Here, the operational-space form is presented but the joint-space form can easily be produced as,



.. math::
    :label: joint_space_acceleration_task_error

    \fjsa_{i} = \left\| \jsrd  - \jsrd_{i}^{\text{des}} \right\|_{2}^{2} \tc


with

.. math::
    :label: joint_space_acceleration_task_error_E

    \Ejsa &= \bmat{I & \0 }

.. math::
    :label: joint_space_acceleration_task_error_f

    \fvecjsa &= \jsrd_{i}^{\text{des}}\tc


where :math:`I` is the identity matrix. Substituting :eq:`joint_space_acceleration_task_error_E` and
:eq:`joint_space_acceleration_task_error_f` into
:eq:`joint_space_acceleration_task_error` gives,



.. math::
    :label: joint_space_acceleration_task_error_in_optvar

    \fjsa_{i} = \left\| \Ejsa \optvar  - \fvecjsa \right\|_{2}^{2} \tp
