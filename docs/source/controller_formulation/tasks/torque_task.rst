
.. _torque_task:

************************
Torque
************************


Finally, it may also be desirable to specify **torque tasks** for purposes of regularization, among other possibilities. As with wrench tasks, torque tasks, can be written simply as,


.. math::
    :label: torque_task

    \torque = \torque^{\text{des}} \tp



To formulate the control objective function, :math:`\ftor`, the square norm of the task error is written,


.. math::
    :label: torque_task_error

    \ftor = \left\| \torque - \torque^{\text{des}} \right\|_{2}^{2} \tc




which can be reformulated in terms of :math:`\optvar` as,


.. math::
    :label: torque_task_error_rewritten

    \ftor = \left\| \bmat{\0 & S^{\top} & \0 } \optvar - \torque^{\text{des}}  \right\|_{2}^{2} \tp



Once again regrouping terms,


.. math::
    :label: torque_task_error_E

    \Etor &= \bmat{\0 & S^{\top} & \0 }

.. math::
    :label: torque_task_error_f

    \fvector &= \torque^{\text{des}}  \tc


the least-squares form of the torque task is written,


.. math::
    :label: torque_task_error_in_optvar

    \ftor = \left\| \Etor \optvar - \fvector \right\|_{2}^{2} \tp
