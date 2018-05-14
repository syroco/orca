End-effector pointing
----------------------------

This tutorials show you how to define the rigth rotation matrix to pointing an target. This section referenced the work of Lucas Joseph, "**Towards X-ray medical imaging with robots in the open:
safety without compromising performances**"

We know that the vector bewteen end-effector and the target can be computed by :

.. math::

    z_o^{des} = \frac{X_T - Xp}{||X_T - X_p||}. 

*  ``X_T`` the position of end-effector expressed in base frame. 
*  ``X_p`` the position of target expressed in base frame. 

.. code-block:: cpp

  void trajectoire(Eigen::Affine3d &cart_pos_ref,int choix, Eigen::Matrix4d T, Eigen::Vector3d Pos_des, Eigen::Matrix3d Rot_des)
  {

      Eigen::Matrix3d rot, rot_base;
      rot_base = Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitX());

      rot =rot_base* Rot_des;
      cart_pos_ref.translation() = Pos_des;
      cart_pos_ref.linear() = rot;
   
   }
Initially, the frame of end-effector is turned by 180° respect to axis x. So, we have to premultiplying the desired rotation by the base rotation. This function ``trajectoire(...)`` compute the desired orientation and position for cartesien acceleration task.

.. code-block:: cpp
    
    int main()
    {
    
    ...
    Pose_laser << 1.,0,0;
    Pose_ee = T.block(0,3,3,1);
    Z_des = (-Pose_ee + Pose_laser) ;
    Z_des = Z_des.normalized();
    Z_des_P = (T.block(0,0,3,3).inverse())*Z_des ; 
    Z_des_P=Z_des_P.normalized();
    axe_rot = Z_des_P.cross(Z_P);
    axe_rot = axe_rot.normalized();
    angle_rot = acos(Z_des_P.dot(Z_P));
    Rot_des =Eigen::AngleAxisd(-angle_rot, axe_rot);  
    ...
    return 0
    }
    
A rotation can just define by an angle and an axis of rotation.     The difference bewteen the position vector of end-effector and the position of target gives us the vector Z desired, and we obtain Z_des expressed in end-effector frame by multiplying this vector by the matrix de transformation bewteen the base frame and end-effector frame. The axis of rotation is obtained by the cross product bewteen Z_des_P and Z of end-effector frame. 
The angle of rotation is given by the dot product. 
Finally, the function ``Eigen::AngleAxisd(-angle_rot, axe_rot); `` compute the desired rotation.