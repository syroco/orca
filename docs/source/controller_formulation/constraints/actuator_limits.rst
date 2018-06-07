.. _actuator_limits:

***************************
Actuator Limit Constraints
***************************

Here, we assume that all articulations are revolute and therefore all actuation limits are torque limits, however, expression of force limits for prismatic joints would be another possibility.
Writing these limits as an inequality provides an upper and lower bound on the amount of torque which can be exerted to accomplish the tasks.



.. math::
    :label: torque_limits

    \torque_{\text{min}} \leq \torque \leq \torque_{\text{max}} \tp



Expressing :ref:`torque_limits` in terms of :math:`\optvar` creates the following linear inequality,



.. math::
    :label: torque_limits_in_optvar_G_and_h

    \underbrace{\bmat{ \0 & S^{\top} & \0 \\ \bs{0} & -S^{\top} & \0 }}_{G^{\torque}}\optvar \leq \underbrace{\bmat{ \torque_{\text{max}} \\ -\torque_{\text{min}} }}_{\bs{h}^{\torque}} \tp
