.. _joint_limits:

*************************
Joint Limit Constraints
*************************


Probably the most common limitation of any robot is the range of motion which each joint can achieve. Whether linear or angular, most joints have a finite range through which they can move thus limiting :math:`\q`. These joint limits can easily be expressed as a inequality on :math:`\q`,



.. math::
    :label: joint_pos_limits

    \q_{\text{min}} \leq \q \leq \q_{\text{max}} \tp




Similarly to these position limits, we can also define limits on the joint velocities and accelerations,



.. math::
    :label: joint_vel_limits

    \jsr_{\text{min}} \leq \jsr \leq \jsr_{\text{max}}

.. math::
    :label: joint_acc_limits

    \jsrd_{\text{min}} \leq \jsrd \leq \jsrd_{\text{max}} \tp





The joint position limits, unlike the torque limits, must be manipulated somewhat in order to be properly expressed in :math:`\optvar`. To formulate this constraint, :math:`\q` needs to be calculated while taking into account a second order prediction of the joint-space movement,



.. math::
    :label: joint_pos_prediction

    \q(t+h) = \q(t) + h\jsr(t) + \frac{h^2}{2}\jsrd(t) \tc




where :math:`h` is the prediction period, which is generally some multiple of the control period. Note that the floating base components of the configuration variable are not subject to articular limits, and their corresponding components in :math:`\q`, :math:`\jsr`, and :math:`\jsrd`, are disregarded in :eq:`joint_pos_prediction`.
Dropping the time dependencies, the limits are written,



.. math::
    :label: joint_pos_limits_with_prediction

    \q_{\text{min}} \leq \q + h\jsr + \frac{h^2}{2}\jsrd \leq \q_{\text{max}}
    \nonumber \\
    \Leftrightarrow
    \frac{2}{h^2}\left[\q_{\text{min}} - (\q + h\jsr) \right] \leq  \jsrd \leq \frac{2}{h^2}\left[\q_{\text{max}} - (\q + h\jsr) \right] \tp





Using :math:`\optvar`, :eq:`joint_pos_limits_with_prediction` can be rewritten as,



.. math::
    :label: joint_pos_limits_in_optvar_G_and_h

    \underbrace
    {
        \bmat{
            I & \0 \\
            -I & \0
        }
    }_{G^{\q}}
    \optvar \leq
    \underbrace
    {
            \frac{2}{h^2}
            \bmat{
                       \q_{\text{max}} - (\q + h\jsr)  \\
                -\left[\q_{\text{min}} - (\q + h\jsr)  \right]
                }
    }_{\bs{h}^{\q}} \tp




From :eq:`joint_pos_limits_in_optvar_G_and_h`, one can of course naturally derive joint velocity and acceleration limits,



.. math::
    :label: joint_vel_limits_in_optvar_G_and_h

    \underbrace
    {
        \bmat{
            I & \0 \\
            -I & \0
        }
    }_{G^{\jsr}}
    \optvar &\leq
    \underbrace
    {
            \frac{1}{h}
            \bmat{
                       \jsr_{\text{max}} - \jsr  \\
                -\left(\jsr_{\text{min}} - \jsr  \right)
                }
    }_{\bs{h}^{\jsr}}

.. math::
    :label: joint_acc_limits_in_optvar_G_and_h

    \underbrace
    {
        \bmat{
            I & \0 \\
            -I & \0
        }
    }_{G^{\jsrd}}
    \optvar &\leq
    \underbrace
    {
            \bmat{
                 \jsrd_{\text{max}} \\
                -\jsrd_{\text{min}}
                }
    }_{\bs{h}^{\jsrd}}
    \tp





The choice of the prediction period, :math:`h`, in the joint-space limits is crucial to the proper functioning of these constraints.
Smaller values of :math:`h` lead to more aggressive approaches to the joint limits, while larger values produce a more conservative treatment.
This variability is due to the fact that the prediction does not take into account the deceleration capabilities of the joints.



.. important::

    To put these constraints into ORCA standard form we have,

    .. math::

        \frac{2}{h^2}\left[ \q_{\text{min}} - (\q + h\jsr) \right]
        &\leq
        \bmat{I & \0} \optvar
        \leq
        \frac{2}{h^2}\left[ \q_{\text{max}} - (\q + h\jsr) \right]
        \\

        \frac{1}{h}\left[ \jsr_{\text{max}} - \jsr \right]
        &\leq
        \bmat{I & \0} \optvar
        \leq
        \frac{1}{h}\left[ \jsr_{\text{max}} - \jsr \right]
        \\

        \jsrd_{\text{max}}
        &\leq
        \bmat{I & \0} \optvar
        \leq
        \jsrd_{\text{max}}
        \\
