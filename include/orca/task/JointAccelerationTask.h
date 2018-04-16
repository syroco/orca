// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
// 
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
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

#pragma once

#include "orca/common/PIDController.h"
#include "orca/task/GenericTask.h"

namespace orca
{
namespace task
{

class JointAccelerationTask : public GenericTask
{
public:
    JointAccelerationTask();

    void setDesired(const Eigen::VectorXd& jnt_pos_des
                , const Eigen::VectorXd& jnt_vel_des
                , const Eigen::VectorXd& jnt_acc_des);

    void updateAffineFunction();

    void resize();
    
    common::PIDController<Eigen::Dynamic>& pid();
    
protected:

    Eigen::VectorXd jnt_pos_des_,jnt_vel_des_,jnt_acc_des_;
    common::PIDController<Eigen::Dynamic> pid_;
};

}
}
