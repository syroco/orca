
.. _wrench_task:

************************
Wrench Task
************************


In order for robots to work properly in their environment, they must be able to interact with it. Not only does this allow the robot to manipulate and modify its environment, but it also allows the robot to exploit the environment to compensate for its underactuation and more generally to dynamically perform complex behaviors.
Walking and balance are two pertinent examples of such behaviors because to achieve them, contact forces with the ground must be properly exploited. For details on this see...

.. todo::

    add citations

.. \citep{Park2005}, \citep{Sentis2010}, and \citep{Righetti2013}.

In order to interact with the environment, **wrench tasks** can be formulated to manage the interaction forces and torques,


.. math::
    :label: wrench_task

    \we_{i} = \we^{\text{des}}_{i} \tp




where :math:`\we^{\text{des}}_{i}` is the desired external wrench to affect, and :math:`\we_{i}` is the wrench applied on the environment. Again, to formulate a control objective function, :math:`\fw_{i}`, the task is rewritten as the squared norm of a task error,


.. math::
    :label: wrench_task_error

    \fw_{i} = \left\| \we_{i} - \we^{\text{des}}_{i} \right\|_{2}^{2} \tp




Rewriting :eq:`wrench_task_error` in terms of :math:`\optvar` gives,


.. math::
    :label: wrench_task_error_rewritten

    \fw_{i} = \left\| \bmat{\0 & S^{\w}_{i} } \optvar - \we^{\text{des}}_{i}  \right\|_{2}^{2} \tc




where :math:`S^{\w}_{i}` is a wrench selection matrix which allows the :math:`i^{\text{th}}` wrench to be controlled. Using,


.. math::
    :label: wrench_task_error_E

    \Ew &= \bmat{\0 & S^{\w}_{i} }


.. math::
    :label: wrench_task_error_f

    \fvecw &= \we^{\text{des}}_{i}
    \tc


:eq:`wrench_task_error_rewritten` can be written as,


.. math::
    :label: wrench_task_error_in_optvar

    \fw_{i} = \left\| \Ew \optvar - \fvecw \right\|_{2}^{2}  \tp
