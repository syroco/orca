// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/


#pragma once

#include "orca/utils/Utils.h"

#include "orca/robot/RobotDynTree.h"

#include "orca/math/Utils.h"

#include "orca/common/CartesianAccelerationPID.h"
#include "orca/common/PIDController.h"
#include "orca/common/Wrench.h"
#include "orca/common/ReturnCode.h"

#include "orca/constraint/DynamicsEquationConstraint.h"
#include "orca/constraint/Contact.h"
#include "orca/constraint/JointLimitConstraint.h"
#include "orca/constraint/JointPositionLimitConstraint.h"
#include "orca/constraint/JointVelocityLimitConstraint.h"
#include "orca/constraint/JointAccelerationLimitConstraint.h"
#include "orca/constraint/JointTorqueLimitConstraint.h"

#include "orca/optim/Controller.h"

#include "orca/task/CartesianTask.h"
#include "orca/task/JointTorqueTask.h"
#include "orca/task/JointAccelerationTask.h"
#include "orca/task/RegularisationTask.h"
#include "orca/task/WrenchTask.h"

namespace orca
{
    namespace all
    {
        using namespace common;
        using namespace optim;
        using namespace task;
        using namespace constraint;
        using namespace robot;
        using namespace math;
        using namespace utils;
    }
}
