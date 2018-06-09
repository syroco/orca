//|  This file is a part of the ORCA framework.
//|
//|  Copyright 2018, Fuzzy Logic Robotics
//|  Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
//|
//|  Main contributor(s): Antoine Hoarau, Ryan Lober, and
//|  Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//|
//|  ORCA is a whole-body reactive controller framework for robotics.
//|
//|  This software is governed by the CeCILL-C license under French law and
//|  abiding by the rules of distribution of free software.  You can  use,
//|  modify and/ or redistribute the software under the terms of the CeCILL-C
//|  license as circulated by CEA, CNRS and INRIA at the following URL
//|  "http://www.cecill.info".
//|
//|  As a counterpart to the access to the source code and  rights to copy,
//|  modify and redistribute granted by the license, users are provided only
//|  with a limited warranty  and the software's author,  the holder of the
//|  economic rights,  and the successive licensors  have only  limited
//|  liability.
//|
//|  In this respect, the user's attention is drawn to the risks associated
//|  with loading,  using,  modifying and/or developing or reproducing the
//|  software by the user in light of its specific status of free software,
//|  that may mean  that it is complicated to manipulate,  and  that  also
//|  therefore means  that it is reserved for developers  and  experienced
//|  professionals having in-depth computer knowledge. Users are therefore
//|  encouraged to load and test the software's suitability as regards their
//|  requirements in conditions enabling the security of their systems and/or
//|  data to be ensured and,  more generally, to use and operate it in the
//|  same conditions as regards security.
//|
//|  The fact that you are presently reading this means that you have had
//|  knowledge of the CeCILL-C license and that you accept its terms.

#pragma once

namespace orca
{
namespace common
{
    // Extracted from qpOASES doc
    // https://www.coin-or.org/qpOASES/doc/3.1/doxygen/MessageHandling_8hpp.html
    enum ReturnCode
    {
          TERMINAL_LIST_ELEMENT = -1
          , SUCCESSFUL_RETURN = 0
          , RET_DIV_BY_ZERO
          , RET_INDEX_OUT_OF_BOUNDS
          , RET_INVALID_ARGUMENTS
          , RET_ERROR_UNDEFINED
          , RET_WARNING_UNDEFINED
          , RET_INFO_UNDEFINED
          , RET_EWI_UNDEFINED
          , RET_AVAILABLE_WITH_LINUX_ONLY
          , RET_UNKNOWN_BUG
          , RET_PRINTLEVEL_CHANGED
          , RET_NOT_YET_IMPLEMENTED
          , RET_INDEXLIST_MUST_BE_REORDERD
          , RET_INDEXLIST_EXCEEDS_MAX_LENGTH
          , RET_INDEXLIST_CORRUPTED
          , RET_INDEXLIST_OUTOFBOUNDS
          , RET_INDEXLIST_ADD_FAILED
          , RET_INDEXLIST_INTERSECT_FAILED
          , RET_INDEX_ALREADY_OF_DESIRED_STATUS
          , RET_ADDINDEX_FAILED
          , RET_REMOVEINDEX_FAILED
          , RET_SWAPINDEX_FAILED
          , RET_NOTHING_TO_DO
          , RET_SETUP_BOUND_FAILED
          , RET_SETUP_CONSTRAINT_FAILED
          , RET_MOVING_BOUND_FAILED
          , RET_MOVING_CONSTRAINT_FAILED
          , RET_SHIFTING_FAILED
          , RET_ROTATING_FAILED
          , RET_QPOBJECT_NOT_SETUP
          , RET_QP_ALREADY_INITIALISED
          , RET_NO_INIT_WITH_STANDARD_SOLVER
          , RET_RESET_FAILED
          , RET_INIT_FAILED
          , RET_INIT_FAILED_TQ
          , RET_INIT_FAILED_CHOLESKY
          , RET_INIT_FAILED_HOTSTART
          , RET_INIT_FAILED_INFEASIBILITY
          , RET_INIT_FAILED_UNBOUNDEDNESS
          , RET_INIT_FAILED_REGULARISATION
          , RET_INIT_SUCCESSFUL
          , RET_OBTAINING_WORKINGSET_FAILED
          , RET_SETUP_WORKINGSET_FAILED
          , RET_SETUP_AUXILIARYQP_FAILED
          , RET_NO_CHOLESKY_WITH_INITIAL_GUESS
          , RET_NO_EXTERN_SOLVER
          , RET_QP_UNBOUNDED
          , RET_QP_INFEASIBLE
          , RET_QP_NOT_SOLVED
          , RET_QP_SOLVED
          , RET_UNABLE_TO_SOLVE_QP
          , RET_INITIALISATION_STARTED
          , RET_HOTSTART_FAILED
          , RET_HOTSTART_FAILED_TO_INIT
          , RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED
          , RET_ITERATION_STARTED
          , RET_SHIFT_DETERMINATION_FAILED
          , RET_STEPDIRECTION_DETERMINATION_FAILED
          , RET_STEPLENGTH_DETERMINATION_FAILED
          , RET_OPTIMAL_SOLUTION_FOUND
          , RET_HOMOTOPY_STEP_FAILED
          , RET_HOTSTART_STOPPED_INFEASIBILITY
          , RET_HOTSTART_STOPPED_UNBOUNDEDNESS
          , RET_WORKINGSET_UPDATE_FAILED
          , RET_MAX_NWSR_REACHED
          , RET_CONSTRAINTS_NOT_SPECIFIED
          , RET_INVALID_FACTORISATION_FLAG
          , RET_UNABLE_TO_SAVE_QPDATA
          , RET_STEPDIRECTION_FAILED_TQ
          , RET_STEPDIRECTION_FAILED_CHOLESKY
          , RET_CYCLING_DETECTED
          , RET_CYCLING_NOT_RESOLVED
          , RET_CYCLING_RESOLVED
          , RET_STEPSIZE
          , RET_STEPSIZE_NONPOSITIVE
          , RET_SETUPSUBJECTTOTYPE_FAILED
          , RET_ADDCONSTRAINT_FAILED
          , RET_ADDCONSTRAINT_FAILED_INFEASIBILITY
          , RET_ADDBOUND_FAILED
          , RET_ADDBOUND_FAILED_INFEASIBILITY
          , RET_REMOVECONSTRAINT_FAILED
          , RET_REMOVEBOUND_FAILED
          , RET_REMOVE_FROM_ACTIVESET
          , RET_ADD_TO_ACTIVESET
          , RET_REMOVE_FROM_ACTIVESET_FAILED
          , RET_ADD_TO_ACTIVESET_FAILED
          , RET_CONSTRAINT_ALREADY_ACTIVE
          , RET_ALL_CONSTRAINTS_ACTIVE
          , RET_LINEARLY_DEPENDENT
          , RET_LINEARLY_INDEPENDENT
          , RET_LI_RESOLVED
          , RET_ENSURELI_FAILED
          , RET_ENSURELI_FAILED_TQ
          , RET_ENSURELI_FAILED_NOINDEX
          , RET_ENSURELI_FAILED_CYCLING
          , RET_BOUND_ALREADY_ACTIVE
          , RET_ALL_BOUNDS_ACTIVE
          , RET_CONSTRAINT_NOT_ACTIVE
          , RET_BOUND_NOT_ACTIVE
          , RET_HESSIAN_NOT_SPD
          , RET_HESSIAN_INDEFINITE
          , RET_MATRIX_SHIFT_FAILED
          , RET_MATRIX_FACTORISATION_FAILED
          , RET_PRINT_ITERATION_FAILED
          , RET_NO_GLOBAL_MESSAGE_OUTPUTFILE
          , RET_DISABLECONSTRAINTS_FAILED
          , RET_ENABLECONSTRAINTS_FAILED
          , RET_ALREADY_ENABLED
          , RET_ALREADY_DISABLED
          , RET_NO_HESSIAN_SPECIFIED
          , RET_USING_REGULARISATION
          , RET_EPS_MUST_BE_POSITVE
          , RET_REGSTEPS_MUST_BE_POSITVE
          , RET_HESSIAN_ALREADY_REGULARISED
          , RET_CANNOT_REGULARISE_IDENTITY
          , RET_CANNOT_REGULARISE_SPARSE
          , RET_NO_REGSTEP_NWSR
          , RET_FEWER_REGSTEPS_NWSR
          , RET_CHOLESKY_OF_ZERO_HESSIAN
          , RET_ZERO_HESSIAN_ASSUMED
          , RET_CONSTRAINTS_ARE_NOT_SCALED
          , RET_INITIAL_BOUNDS_STATUS_NYI
          , RET_ERROR_IN_CONSTRAINTPRODUCT
          , RET_FIX_BOUNDS_FOR_LP
          , RET_USE_REGULARISATION_FOR_LP
          , RET_UPDATEMATRICES_FAILED
          , RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED
          , RET_UNABLE_TO_OPEN_FILE
          , RET_UNABLE_TO_WRITE_FILE
          , RET_UNABLE_TO_READ_FILE
          , RET_FILEDATA_INCONSISTENT
          , RET_OPTIONS_ADJUSTED
          , RET_UNABLE_TO_ANALYSE_QPROBLEM
          , RET_NWSR_SET_TO_ONE
          , RET_UNABLE_TO_READ_BENCHMARK
          , RET_BENCHMARK_ABORTED
          , RET_INITIAL_QP_SOLVED
          , RET_QP_SOLUTION_STARTED
          , RET_BENCHMARK_SUCCESSFUL
          , RET_NO_DIAGONAL_AVAILABLE
          , RET_DIAGONAL_NOT_INITIALISED
          , RET_ENSURELI_DROPPED
          , RET_SIMPLE_STATUS_P1
          , RET_SIMPLE_STATUS_P0
          , RET_SIMPLE_STATUS_M1
          , RET_SIMPLE_STATUS_M2
          , RET_SIMPLE_STATUS_M3
    };
} // namespace common
} // namespace orca
