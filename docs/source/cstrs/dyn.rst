Dynamics Equation
=================

* **Control variable** : X (whole optimisation vector)
* **Type** : Equality constraint
* **Size** : :math:`ndof \times size(X)`

.. math::
    
    \begin{bmatrix}
    - M &&
    S_{\tau} &&
    J_{^{e}w}
    \end{bmatrix} X
    = C + G

.. code-block:: c++

    orca::constraint::DynamicsEquationConstraint dyn_eq;
    dyn_eq.loadRobotModel( urdf );
    dyn_eq.setGravity( Eigen::Vector3d(0,0,-9.81) );
    dyn_eq.update(); // <-- Now initialized
    
    dyn_eq.activate(); // <-- Now activated 
    dyn_eq.insertInProblem(); // <-- Now part of the optimisation problem 
