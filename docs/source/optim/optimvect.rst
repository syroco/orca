Optimisation Vector
===================

The optimisation vector in the quadratic problem is written as follows : 

.. math::
    
    X =
    \begin{pmatrix}
    \dot{\nu}^{fb}\\
    \dot{\nu}^{j}\\ 
    \tau^{fb}\\ 
    \tau^{j}\\ 
    ^{e}w_{0}\\
    \vdots\\
    ^{e}w_{n}\\
    \end{pmatrix}


* :math:`\dot{\nu}^{fb}`: Floating base joint acceleration (:math:`6 \times 1`)
* :math:`\dot{\nu}^{j}` : Joint space acceleration (:math:`n_{dof} \times 1`)
* :math:`\tau^{fb}`     : Floating base joint torque (:math:`6 \times 1`)
* :math:`\tau^{j}`      : Joint space joint torque (:math:`n_{dof} \times 1`)
* :math:`^{e}w_n`       : External wrench (:math:`6 \times 1`)


In ORCA those are called `Control variables` and should be used to define every task and constraint.
In addition to those necessary variables, you can specify also a combination :

* :math:`\dot{\nu}`: Generalised joint acceleration, concatenation of :math:`\dot{\nu}^{fb}` and :math:`\dot{\nu}^{j}` (:math:`6+n_{dof} \times 1`)
* :math:`\tau`     : Generalised joint torque, concatenation of :math:`\dot{\tau}^{fb}` and :math:`\dot{\tau}^{j}` (:math:`6+n_{dof} \times 1`)
* :math:`X`      : The whole optimisation vector (:math:`6 + n_{dof} + 6 + n_{dof} + n_{wrenches}6 \times 1`)
* :math:`^{e}w`          : External wrenches (:math:`n_{wrenche} 6 \times 1`)