.. _multi_obj_optim:


*******************************
Multi-Objective Optimization
*******************************


Objective functions represent the intentions of the problem designer: what meaningful quantity or measure is to be minimized to best solve some issue.
As is often the case, there may be more than one quantity or measure which must be minimized and therefore multiple objective functions are combined together.
When multiple objective functions, :math:`f_{i}(\optvar)`, are considered simultaneously, a **multi-objective optimization** problem (a.k.a. multicriteria, multicriterion, or Pareto optimization) is created. One common method of solving multi-objective optimization problems is through \textit{scalarization}. Scalarization is the process of combining of multiple objective costs into one scalar cost. There are a multitude of scalarization techniques but weighted summation is of the most common,



.. math::
    :label: multi_objective_qp

    \underset{\optvar}{\argmin} \sum_{i=1}^{n_{o}} w_{i} f_{i}(\optvar) = \sum_{i=1}^{n} w_{i} \left\| E_{i}\optvar - \fvec_{i} \right\|^{2}_{2}  \tp




In :eq:`multi_objective_qp`, :math:`n_{o}` is the total number of objective functions. This scalarization can be written compactly by concatenating the individual objectives as,



.. math::
    :label: weighted_qp

    \underset{\optvar}{\argmin} \quad \left\| E_{w}\optvar - \fvec_{w} \right\|^{2}_{2}




where



.. math::
    :label: weighted_qp_E_and_f

    E_{w} =
    \bmat{ \sqrt{w_{1}}E_{1} \\ \sqrt{w_{2}}E_{2} \\ \vdots \\ \sqrt{w_{n}}E_{n_{o}} }
    \quad \text{and} \quad
    \fvec_{w} =
    \bmat{ \sqrt{w_{1}}\fvec_{1} \\ \sqrt{w_{2}}\fvec_{2} \\ \vdots \\ \sqrt{w_{n}}\fvec_{n_{o}} }
    \tp




Each weight, :math:`w_{i} \geq 0`, dictates the relative importance of its objective :math:`f_i(\optvar)` and therefore its impact on the solution. In :eq:`weighted_qp` the weights are assumed to be scalars, but it is also possible to use matrices of different weights as long as they remain positive semi-definite.

As an alternative to scalarization, the objective functions can be minimized hierarchically in order of importance to ensure that the most important objective(s) are minimized as much as possible without influence of the lower priority objectives.
This is known as **lexicographic optimization** in multi-objective optimization.
To achieve this, the objectives are treated individually as a cascade of QPs where the solutions are reused as equality constraints in the subsequent QP minimizations.




.. _prioritization_strategies:

Resolution (Prioritization) Strategies for Whole-Body Control
===================================================================


If multiple task objective functions are combined (using operations that preserve convexity) in the resolution of the control problem, then they can be performed simultaneously.
In these cases, it is important to select a strategy for the resolution of the optimization problem. In turn, the strategy determines how tasks interact/interfere with one another.
The two prevailing methods for dealing with multiple tasks are hierarchical and weighted prioritization.





.. _Hierarchical_Prioritization:

Hierarchical Prioritization
---------------------------------------


In **hierarchical prioritization**, the tasks are organized by order of importance in discrete levels.
Each task error is minimized in descending order of its importance and the solution to the optimization problem is then used in the equality constraints for the proceeding optimizations.


.. admonition:: Hierarchical Prioritization Algorithm

    .. math::

        \text{for}& \quad \left(i =1 \dots \nt \right)
        \\[3mm]
        &\begin{aligned}
        \optvar^{*}_{i} = \underset{\optvar}{\argmin} &\quad \ft_{i}(\optvar) + w_{0}\ft_{0}(\optvar) \\
          \text{s.t.} 			&\quad G\optvar \leq \bs{h} \\
                                &\quad A_{i}\optvar = \bs{b}_{i}
        \\
        A_{i+1}& \leftarrow \bmat{ A_{i} \\ E_{i} }
        \\
        \bs{b}_{i+1}& \leftarrow \bmat{ \bs{b}_{i} \\ \optvar^{*}_{i}}
        \\
        \optvar^{*}& \leftarrow \optvar^{*}_{i}
        \end{aligned}
        \\[3mm]
        \text{return}& \quad \optvar^*


This algorithm is tantamount to null-space projection in the dynamic domain; however, inequality constraints can be accounted for.
As a note, the regularization term, :math:`w_{0}\ft_{0}(\x)`, in each optimization cascade serves to remove solution redundancy when the objective function has a null space, but this redundancy is necessary for executing the subsequent tasks. The operation, :math:`A_{i+1} \leftarrow \bmat{ A_{i} \\ E_{i} }`, propagates the null space of the objective function, which has just been solved, to the proceeding objective functions through the equality constraint.

Resolving the whole-body control problem hierarchically has the benefit of strictly ensuring the optimization of one task error over another; however, it makes task transitioning and blending more difficult. Using continuous, or soft, priorities can alleviate some of these issues.



.. _Weighted_Prioritization:

Weighted Prioritization
---------------------------------------

In multi-objective optimization, task weights dictate where, on the Pareto front of solutions, the QP calculates an optimum. Consequently, the optimum found favors the minimization of tasks with higher weights. This affords a method of prioritization, which ensures that critical tasks, such as those for balance, are preferentially accomplished, in situations where other less-critical tasks, such as a reach, have conflicting optima.

.. admonition:: Weighted Prioritization Algorithm

    .. math::

        \optvar^* = \underset{\optvar}{\argmin} 	&\quad \displaystyle\sum_{i=1}^{\nt}  w_{i} \ft_{i}(\optvar) + w_{0}\ft_{0}(\optvar) \\
              \text{s.t.} 	&\quad G\optvar \leq \bs{h} \\
                 		&\quad A\optvar = \bs{b}  \tp
        \\[3mm]
        \text{return}& \quad \optvar^*


However, using continuous priorities between tasks cannot guarantee that the tasks will not interfere with one another.

.. important::

    In fact, each task will assuredly impact the ensemble but that impact can be rendered numerically negligible.








Hybrid Schemes
-------------------------


It can be seen that the weighted strategy is a subset of the hierarchical strategy, by observing that each level in a hierarchical scheme can be solved as a weighted problem. This **hybrid prioritization strategy** can provide the best of both hierarchical and weighted methods, but at the cost of increase implementation and computational complexity.



Generalized Hierarchical Prioritization
------------------------------------------

In addition to the simple mixing of weights and hierarchies, continuous generalized projection schemes are developed by \citep{Liu2016}. These methods allow priorities to continuously vary from weighted to purely hierarchical through scalar values. Such approaches can provide smooth transitions between tasks, as is common in complex activities such as walking, but require substantially more computation time than purely weighted or hierarchical methods.


Resolution Strategies in ORCA
=======================================

ORCA provides three strategies for resolving a multi-objective QP which containts multiple tasks and/or constraints.

#. ``OneLevelWeighted`` (weighted prioritization)
#. ``MultiLevelWeighted`` (hybrid prioritization)
#. ``Generalized`` (generalized hierarchical prioritization)

.. note::

    these strategies are in the namespace ``orca::optim::ResolutionStrategy``

Each of these strategies is detailed in the following sections.
