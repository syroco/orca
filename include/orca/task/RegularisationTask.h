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

#include "orca/task/GenericTask.h"

namespace orca
{
namespace task
{
    template<optim::ControlVariable C> class RegularisationTask : public GenericTask
    {
    public:
        using Ptr = std::shared_ptr<RegularisationTask<C> >;
        
        const double DefaultWeight = 1.e-4;

        RegularisationTask(const std::string& name)
        : GenericTask(name,C)
        {
            this->setRampDuration(0);
            this->setWeight(DefaultWeight);
        }
    protected:
        void onActivation()
        {}
        void onUpdateAffineFunction(double current_time, double dt)
        {}
        void onDeactivation()
        {}
        void onResize()
        {
            const int sizeofvar = this->getProblem()->getSize(this->getControlVariable());
            int old_cols = euclidianNorm().cols();

            if(old_cols != sizeofvar)
            {
                euclidianNorm().resize(sizeofvar,sizeofvar);
                euclidianNorm().A().setIdentity();
                euclidianNorm().b().setZero();
                euclidianNorm().computeQuadraticCost();
            }
        }
    };

    typedef RegularisationTask<optim::ControlVariable::GeneralisedAcceleration>   GeneralisedAccelerationRegularisationTask;
    typedef RegularisationTask<optim::ControlVariable::JointAcceleration>         JointAccelerationRegularisationTask;
    typedef RegularisationTask<optim::ControlVariable::GeneralisedTorque>         GeneralisedTorqueRegularisationTask;
    typedef RegularisationTask<optim::ControlVariable::JointTorque>               JointTorqueRegularisationTask;
    typedef RegularisationTask<optim::ControlVariable::ExternalWrench>            WrenchRegularisationTask;
    typedef RegularisationTask<optim::ControlVariable::X>                         GlobalRegularisationTask;
} // namespace task
} // namespace orca

ORCA_REGISTER_CLASS(orca::task::RegularisationTask<orca::optim::ControlVariable::X>,0)
ORCA_REGISTER_CLASS(orca::task::RegularisationTask<orca::optim::ControlVariable::GeneralisedAcceleration>,1)
ORCA_REGISTER_CLASS(orca::task::RegularisationTask<orca::optim::ControlVariable::JointAcceleration>,2)
ORCA_REGISTER_CLASS(orca::task::RegularisationTask<orca::optim::ControlVariable::GeneralisedTorque>,3)
ORCA_REGISTER_CLASS(orca::task::RegularisationTask<orca::optim::ControlVariable::JointTorque>,4)
ORCA_REGISTER_CLASS(orca::task::RegularisationTask<orca::optim::ControlVariable::ExternalWrench>,5)
