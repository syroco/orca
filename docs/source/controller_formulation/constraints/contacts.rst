.. _contacts:

***********************
Contacts
***********************


When a robot interacts with its environment, it does so through contacts. These contacts can be **unilateral contacts**, or **bilateral contacts**. Simply put, unilateral contacts are those the robot can only push, e.g. foot contact with the floor, and bilateral contacts are those which allow the robot to push or pull, e.g. gripping the rung of a ladder.

.. todo::

    add citations: Following the formulations in \citep{Salini2011} and \citep{Saab2013}

For unilateral contact constraints, a linearized approximation of the Coulomb friction cone is employed. A friction contact constraint in the controller must ensure that the linear velocity at the contact point is zero,


.. math::
    :label: friction_no_motion

    \Jf_{i}(\q)\jsrd + \Jfd_{i}(\q, \jsr)\jsr = \0 \tc



and that the wrench remains within a linearized approximation of a friction cone,


.. math::
    :label: friction_linear_cone

    \Cf_{i}\wf_{i} \leq \0 \tp



In :eq:`friction_no_motion`, :math:`\Jf` and :math:`\Jfd` contain the linear components of the :math:`i^{\text{th}}` contact Jacobian. In :eq:`friction_linear_cone`, :math:`\Cf_{i}` is a matrix which linearly approximates the second-order norm cone,


.. math::
    :label: friction_cone

    \left\| \wf_{i} - (\wf_{i} \cdot \bs{\hat{n}}_{i})\bs{\hat{n}}_{i} \right\|_{2} \leq \mu_{i}(\wf_{i} \cdot \bs{\hat{n}}_{i}) \tc



where :math:`\wf_{i}` is are the force components of the :math:`i^{\text{th}}` contact wrench, :math:`\bs{\hat{n}}_{i}` is the normal vector of the contact, and :math:`\mu_{i}` is the friction coefficient. Finally, expressing these two constraints in terms of :math:`\optvar`, and defining :math:`\wf_{i} = S^{F}_{i}\optvar`, gives the following coupled equality and inequality constraints,


.. math::
    :label: friction_equality_constraint

    \underbrace
    {
        \bmat
        {
            \Jf_{i}(\q) & \0
        }
    }_{A^{\w}}
    \optvar &=
    \underbrace
    {
        -\Jfd_{i}(\q, \jsr)\jsr
    }_{\bs{b}^{\w}}

.. math::
    :label: friction_inequality_constraint

    \underbrace
    {
        \bmat
        {
            \0 & \Cf_{i}S^{F}_{i}
        }
    }_{G^{\w}}
    \optvar &\leq
    \underbrace
    {
        \0
    }_{\bs{h}^{\w}}
     \tc



where :math:`S^{F}_{i}` selects the :math:`i^{\text{th}}` contact force vector.
Equations :eq:`friction_equality_constraint` and :eq:`friction_inequality_constraint` are valid for a single contact point. For surface contacts, e.g. a foot sole, multiple points on the surface can be used for friction contact constraints --- usually the four corners of the foot.
Equation :eq:`friction_equality_constraint` introduces 3 equality constraints for the linear velocity of the contact point.
The number of inequality constraints introduced by :eq:`friction_inequality_constraint` depends on the number of polygon edges used to approximate the friction cone.
Here, 6 edges are used, and because of symmetry, this introduces 3 inequality constraints per contact to the \ctrller{}.

For bilateral contacts, it is sufficient to ensure no relative motion between the two links, :math:`i` and :math:`j` in contact. It should be noted that here a link can be some part of the environment for which a kinematic model exists. To ensure no motion between the links, the following relationship must be true,


.. math::
    :label: bilateral_no_motion

    \left( J_{i}(\q) - J_{j}(\q) \right)\jsrd + \left( \dot{J}_{i}(\q,\jsr) - \dot{J}_{j}(\q,\jsr) \right)\jsr = \0 \tc



where :math:`J_{i}(\q)`, :math:`\dot{J}_{i}(\q,\jsr)`, :math:`J_{j}(\q)`, and :math:`\dot{J}_{j}(\q,\jsr)`, are the Jacobians and their derivatives for the :math:`i`\textsuperscript{th} and :math:`j`\textsuperscript{th} links respectively.
Putting :eq:`bilateral_no_motion` in terms of :math:`\optvar` produces,


.. math::
    :label: bilateral_equality_constraint

    \underbrace
    {
        \bmat
        {
            \left( J_{i}(\q) - J_{j}(\q) \right) & \0
        }
    }_{A^{bc}}
    \optvar =
    \underbrace
    {
        -\left( \dot{J}_{i}(\q,\jsr) - \dot{J}_{j}(\q,\jsr) \right)\jsr
    }_{\bs{b}^{bc}}
    \tp
