***********************
Optimization Vector
***********************

In :ref:`dynamics` we expressed the equations of motion as an affine function of our optimization variable, :math:`\optvar`. Here, we look at each component in :math:`\optvar` and detail its meaning, position in the overall vector, and dimensions.

.. math::

    \optvar =
    \bmat{
    \jsrd_{fb}\\
    \jsrd_{j}\\
    \torque_{fb}\\
    \torque_{j}\\
    \we_{0}\\
    \vdots\\
    \we_{n}\\
    }


* :math:`\jsrd_{fb}` : Floating base joint acceleration (:math:`6 \times 1`)
* :math:`\jsrd_{j}`  : Joint space acceleration (:math:`n_{\dof} \times 1`)
* :math:`\torque_{fb}`      : Floating base joint torque (:math:`6 \times 1`)
* :math:`\torque_{j}`       : Joint space joint torque (:math:`n_{\dof} \times 1`)
* :math:`\we_{n}`        : External wrench (:math:`6 \times 1`)

Each of these variables are termed **Control Variables** in ORCA and are used to define every task and constraint.

These variables can of course be combined for convenience:

* :math:`\jsrd` : Generalised joint acceleration, concatenation of :math:`\jsrd_{fb}` and :math:`\jsrd_{j}` (:math:`6+n_{\dof} \times 1`)
* :math:`\torque`      : Generalised joint torque, concatenation of :math:`\torque_{fb}` and :math:`\torque_{j}` (:math:`6+n_{\dof} \times 1`)
* :math:`\we`     : External wrenches (:math:`n_{\text{wrenches}} 6 \times 1`)
* :math:`\optvar`         : The whole optimization vector (:math:`6 + n_{\dof} + 6 + n_{\dof} + n_{wrenches}6 \times 1`)


With our optimization varible well defined, we can now formulate the optimization problem.
