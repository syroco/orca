
.. _cart_acc_task:

************************
Cartesian Acceleration
************************


Probably the most important, if not most prevalent, task is to move a link on the robot from one pose to another. Typically it is the end-effector(s) which are of interest.
These tasks, which are generally expressed as desired positions or orientations, are converted to **acceleration tasks**, through means of task servoing. More details on task servoing are provided in :ref:`task_servoing`.
Once given a desired operational-space acceleration for a link, :math:`\acc^{\text{des}}_{i}`, an acceleration task consists in finding the joint-space values which produce :math:`\acc^{\text{des}}_{i}`,



.. math::
    :label: acceleration_task

    \acc^{\text{des}}_{i} = J_{i}(\q)\jsrd + \dot{J}_{i}(\q,\jsr)\jsr \tc


where :math:`J_{i}(\q)` and :math:`\dot{J}_{i}(\q,\jsr)` are the link Jacobian and its derivative. For the control objective, one simply rewrites the task as an error which must be minimized,



.. math::
    :label: acceleration_task_error

    \fa_{i} = \left\| J_{i}(\q)\jsrd + \dot{J}_{i}(\q,\jsr)\jsr - \acc_{i}^{\text{des}} \right\|_{2}^{2} \tp


Using the squared :math:`l^{2}`-norm produces a quadratic error term, which defines the objective function :math:`\fa_{i}` to be minimized. The objective function :math:`\fa_{i}` is then rewritten in terms of the optimization variable, :math:`\optvar`,



.. math::
    :label: acceleration_task_error_rewritten

    \fa_{i} = \left\| \bmat{J_{i}(\q) & \0 } \optvar - \left(\acc_{i}^{\text{des}} - \dot{J}_{i}(\q,\jsr)\jsr \right)  \right\|_{2}^{2} \tp


In :eq:`acceleration_task_error_rewritten` the term :math:`\0` represents a matrix of zeros. Regrouping terms as,


.. math::
    :label: acceleration_task_error_E

    \Ea &= \bmat{J_{i}(\q) & \0 }

.. math::
    :label: acceleration_task_error_f

    \fveca &= \acc_{i}^{\text{des}} - \dot{J}_{i}(\q,\jsr)\jsr \tc


allows :eq:`acceleration_task_error_rewritten` to be written in the classical least-squares form as,



.. math::
    :label: acceleration_task_error_in_optvar

    \fa_{i} = \left\| \Ea \optvar - \fveca \right\|_{2}^{2}  \tp


The dependencies of :math:`\Ea` and :math:`\fveca` have been removed for brevity.


.. math::

    w_{task} . \lVert \mathbf{E}x + \mathbf{f} \rVert_{W_{norm}}

.. math::

    \underset{n\times 1}{\mathrm{Y}} =  \underset{n\times p}{X} \times
    \underset{p\times 1}{\theta} + \underset{n\times 1}{\varepsilon}
