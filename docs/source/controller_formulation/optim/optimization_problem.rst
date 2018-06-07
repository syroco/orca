.. _optimization_problem:

**************************************
The Optimization Problem
**************************************

Returning to our generic representation of a whole-body controller presented in :ref:`overview`,


.. math::
    :label: generic_whole_body_controller_again

    \underset{\optvar}{\argmin} &\quad \ft(\optvar)   \\
    \text{s.t.}            &\quad G\optvar \leq \bs{h}  \\
                           &\quad A\optvar = \bs{b}  \tc

we make some important assumptions about the structure of the problem. Firstly, we make the assumtion that our control problem is continous and has **size=:math:`n`**, i.e. :math:`\optvar \in \R^{n}`. Next we impose that :math:`\ft(\optvar)` be quadratic in :math:`\optvar`, leaving us with an unconstrained **Quadratic Program**, or QP:

.. math::
    :label: qp

    \underset{\optvar}{\argmin} \quad f(\optvar)
    &= \frac{1}{2} \optvar^{\top}H\optvar + \bs{g}^{\top}\optvar + r  \\
    &= \optvar^{\top}(E^{\top}E)\optvar - 2(E^{\top}\fvec)^{\top}\optvar + \fvec^{\top}\fvec \\
    &= \left\| E\optvar - \fvec \right\|^{2}_{2} \tc



In :eq:`qp`, the first line is the classical formulation of a QP:

* :math:`\optvar` the optimization vector
* :math:`H` the hessian matrix (:math:`n \times n`)
* :math:`\bs{g}` the gradient vector (:math:`n \times 1`)
* :math:`E` the linear matrix of the affine function (:math:`n \times n`)

* :math:`f` the origin vector (:math:`n \times 1`)

The last line of :eq:`qp`, :math:`\left\| E\optvar - \fvec \right\|^{2}_{2}`, is the least-squares formulation.
We will continue using the least squares version, which admits an analytical minimum-norm solution, :math:`\optvar^{*}`, in the unconstrained case.

.. math::
    :label: analytical_least_squares_solution

    \optvar^{*} = \underset{\optvar}{\argmin} \left\| E\optvar - \fvec \right\|^{2}_{2} = E^{\dagger}\fvec \tc


where :math:`E^{\dagger}` is the Moore-Penrose pseudoinverse of the :math:`E` matrix. This solution will be found assuming the rank of the linear system is consistent.

Adding an affine equality constraint produces a constrained least squares problem,

.. math::
    :label: constrained_least_squares

    \underset{\optvar}{\argmin} &\quad \left\| E\optvar - \fvec \right\|^{2}_{2}   \\
    \text{s.t.}            &\quad A\optvar = \bs{b}  \tc

which can be solved analytically, assuming a solution exists, using the **Karush Kuhn Tucker (KKT) equations**,

.. math::
    :label: KKT_equations

    \underbrace{\bmat{E^{\top}E & A^{\top} \\ A & \0}}_{\text{KKT Matrix}}
    \bmat{\optvar \\ \bs{z}}
    &=
    \bmat{E^{\top}\fvec \\ \bs{b}}
    \\
    \Leftrightarrow
    \bmat{\optvar \\ \bs{z}}
    &= \bmat{E^{\top}E & A^{\top} \\ A & \0}^{-1}
    \bmat{E^{\top}\fvec \\ \bs{b}}
    \tc


where :math:`\bs{z}` is the solution to the dual problem and contains the **Lagrange multipliers**.


Adding an affine inequality constraint to the problem produces the following QP,

.. math::
    :label: qp_with_constraints

    \underset{\optvar}{\argmin} &\quad \left\| E\optvar - \fvec \right\|^{2}_{2}   \\
    \text{s.t.}            &\quad A\optvar = \bs{b}  \\
                           &\quad G\optvar \leq \bs{h}  \tp



Equation :eq:`qp_with_constraints` can no longer be solved analytically and one must use numerical methods such as interior point, or active set methods.

.. note::

    For more details on convex optimization, check out Stephen Boyd's book: http://web.stanford.edu/~boyd/cvxbook/

Resolution of :eq:`qp_with_constraints` with a numerical solver, such as ``qpOASES``, will provide a globally optimal solution for :math:`\optvar^{*}` provided that the constraint equations are consistent, i.e. the set of possible solutions is not empty.


Objective Function Implementation
====================================

Within ORCA the QP objective function is formulated as,

.. Hessian_.noalias() = SelectionVector.asDiagonal() * Weight * A.transpose() * A ;

.. Gradient_.noalias() =  2.0 * SelectionVector.asDiagonal() * Weight * A.transpose() * b ;


.. math::

    \frac{1}{2} \optvar^{\top}H\optvar + \bs{g}^{\top}\optvar \\
    \Leftrightarrow \optvar^{\top}(E^{\top}E)\optvar - 2(E^{\top}\fvec)^{\top}\optvar  \nonumber \\
    \Leftrightarrow \optvar^{\top}(S W E^{\top}E)\optvar - 2 S W (E^{\top}\fvec)^{\top}\optvar  \nonumber \\
    \left\| E\optvar - \fvec \right\|^{2}_{W}

* :math:`W` the weight of the euclidean norm (:math:`n \times n`)


.. note::

    :math:`r = \fvec^{\top}\fvec` is dropped from the objective function because it does not change the optimal solution of the QP.



Constraint Implementation
============================



.. math::

    \underset{\optvar}{\argmin} &\quad \frac{1}{2} \optvar^{\top}H\optvar + \bs{g}^{\top}\optvar \\
    \text{s.t.} & lb \leq C\optvar \leq ub

* :math:`A` the constraint matrix (:math:`n \times n`)
* :math:`lb` and ``ub`` the lower and upper bounds of ``x`` (:math:`n \times 1`)
* :math:`lbA` and ``ubA`` the lower and upper bounds of ``A`` (:math:`n \times 1`)
