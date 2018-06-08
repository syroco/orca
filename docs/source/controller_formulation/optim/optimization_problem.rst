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

we make some important assumptions about the structure of the problem. Firstly, we make the assumtion that our control problem is continous and has size = :math:`n`, i.e. :math:`\optvar \in \R^{n}`. Next we impose that :math:`\ft(\optvar)` be quadratic in :math:`\optvar`, leaving us with an unconstrained **Quadratic Program**, or QP:

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

    For more details on convex optimization, check out Boyd and Vandenberghe's book: http://web.stanford.edu/~boyd/cvxbook/

Resolution of :eq:`qp_with_constraints` with a numerical solver, such as ``qpOASES``, will provide a globally optimal solution for :math:`\optvar^{*}` provided that the constraint equations are consistent, i.e. the set of possible solutions is not empty.


Objective Function Implementation
====================================

Within ORCA the QP objective function is formulated as a weighted Euclidean norm of an affine function,

.. math::
    :label: weighted_norm

    \left\| E\optvar - \fvec \right\|^{2}_{W} \Leftrightarrow \left\| \sqrt{W} \left( E\optvar - \fvec \right) \right\|^{2}

In :eq:`weighted_norm`, :math:`W` is the weight of the euclidean norm (:math:`n \times n`) and must be a positive symmetric definite matrix.

In ORCA, :math:`W` is actually composed of two components, the norm weighting :math:`W'` and the selection matrix, :math:`S`,

.. math::
    :label: weighting_matrix

    W = SW'

:math:`S` is a matrix with either 1's or 0's on the diagonal which allows us to ignore all or parts of the affine function we are computing. Concretely this means we can ignore components of the task error. More information on tasks is provided in the :ref:`tasks` section.

.. admonition:: For example...

    For a Cartesian position task, setting the low 3 entries on the diagonal of :math:`S` to 0 allows us to ignore orientation errors.

For practicality's sake we set :math:`S` from a vector with the function ``setSelectionVector(const Eigen::VectorXd& s)``, which creates a diagonal matrix from ``s``.

Given :math:`W` from :eq:`weighting_matrix`, the hessian and gradient are calculated as,


.. math::

    \frac{1}{2} \optvar^{\top}H\optvar + \bs{g}^{\top}\optvar \\
    \Leftrightarrow \optvar^{\top}(E^{\top}WE)\optvar - 2 (WE^{\top}\fvec)^{\top}\optvar


.. note::

    :math:`r = \fvec^{\top}\fvec` is dropped from the objective function because it does not change the optimal solution of the QP.



In the code, these calculations can be found in ``WeightedEuclidianNormFunction``:

.. code-block:: c++

    void WeightedEuclidianNormFunction::QuadraticCost::computeHessian(const Eigen::VectorXd& SelectionVector
                                                    , const Eigen::MatrixXd& Weight
                                                    , const Eigen::MatrixXd& A)
    {
        Hessian_.noalias() = SelectionVector.asDiagonal() * Weight * A.transpose() * A ;
    }

    void WeightedEuclidianNormFunction::QuadraticCost::computeGradient(const Eigen::VectorXd& SelectionVector
                                                    , const Eigen::MatrixXd& Weight
                                                    , const Eigen::MatrixXd& A
                                                    , const Eigen::VectorXd& b)
    {
        Gradient_.noalias() =  2.0 * SelectionVector.asDiagonal() * Weight * A.transpose() * b ;
    }





Constraint Implementation
============================


Constraints are written as double bounded linear functions,

.. math::

    \bs{lb} \leq C\optvar \leq \bs{ub} \tp

* :math:`C` the constraint matrix (:math:`n \times n`)
* :math:`\bs{lb}` and :math:`\bs{ub}` the lower and upper bounds of :math:`C\optvar` (:math:`n \times 1`)

Thus to convert our standard affine constraint forms we have the following relationships:

.. math::

    A\optvar = \bs{b} \Leftrightarrow \bs{b} \leq A\optvar \leq \bs{b}


.. math::

    G\optvar \leq \bs{h} \Leftrightarrow \bmat{G\optvar \\ -G\optvar} \leq \bmat{\bs{ub_{h}} \\ -\bs{lb_{h}}} \Leftrightarrow \bs{lb_{h}} \leq G\optvar \leq \bs{ub_{h}}



ORCA QP
==============

In ORCA the full QP is expressed as,

.. math::

    \underset{\optvar}{\argmin} &\quad \frac{1}{2} \optvar^{\top}H\optvar + \bs{g}^{\top}\optvar \\
    \text{s.t.} &\quad \bs{lb} \leq C\optvar \leq \bs{ub} \tc

In the next sections we show how to formulate the different task and constraint types one might need to control a robot.
In section :ref:`multi_obj_optim`, we show how to combine multiple objective functions (tasks) in one controller allowing us to exploit the redundancy of the system.

.. note::

    Multiple constraints can be combined through vertical concatenation of their matrices and vectors. I.e.

    .. math::

        \bmat{\bs{lb}_{1} \\ \bs{lb}_{2} \\ \vdots \\ \bs{lb}_{n_{C}} }
        \leq
        \bmat{C_{1} \\ C_{2} \\ \vdots \\ C_{n_{C}} } \optvar
        \leq
        \bmat{\bs{ub}_{1} \\ \bs{ub}_{2} \\ \vdots \\ \bs{ub}_{n_{C}} }
