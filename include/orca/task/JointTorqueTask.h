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
#include "orca/common/PIDController.h"

namespace orca
{
namespace task
{

class JointTorqueTask : public GenericTask
{
public:
    JointTorqueTask(const std::string& name);

    void setDesired(const Eigen::VectorXd& jnt_trq_des);

    void setCurrent(const Eigen::VectorXd& jointTorque);

    common::PIDController::Ptr pid();
    
    void setFeedforward(const Eigen::VectorXd& ff);
    const Eigen::VectorXd& getFeedforward() const;
    
protected:
    virtual void onUpdateAffineFunction(double current_time, double dt);
    virtual void onResize();
    virtual void onActivation();
private:
    common::Parameter<Eigen::VectorXd> feedforward_;
    common::Parameter<common::PIDController::Ptr> pid_ = std::make_shared<common::PIDController>("pid");
private:
    bool desired_set_ = false;
    Eigen::VectorXd jnt_trq_des_,current_jnt_trq_;
};

}
}
