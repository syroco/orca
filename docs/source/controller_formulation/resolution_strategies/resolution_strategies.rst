.. _resolution_strategies:


*******************************
Resolution Strategies
*******************************



OneLevelWeighted
MultiLevelWeighted
Generalized

Given n_t tasks, the **overall cost function** is such that:

.. math::

    \frac{1}{2}x^tHx + x^tg = \frac{1}{2} \sum_{i=1}^{n_t}  w_{task,i}  \lVert \mathbf{E}_ix + \mathbf{f}_i \rVert_{W_{norm,i}}^2

Constraints are written as **double bounded linear function** :

.. math::

    lb_C \leq Cx \leq ub_C

* :math:`C` the constraint matrix (:math:`n \times n`)
* :math:`lbC` and :math:`ubC` the lower and upper bounds of :math`A` (:math:`n \times 1`)
