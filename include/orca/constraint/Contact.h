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

#include "orca/common/Wrench.h"
#include "orca/constraint/LinearizedCoulombConstraint.h"
#include "orca/constraint/ContactExistenceConditionConstraint.h"

namespace orca
{
namespace constraint
{

class Contact : public common::TaskBase
{

public:
    Contact(const std::string& name);

    void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot);

    bool loadRobotModel(const std::string& file_url);

    void setName(const std::string& name);

    void setBaseFrame(const std::string& base_ref_frame);

    void setControlFrame(const std::string& control_frame);

    const std::string& getBaseFrame() const;

    const std::string& getControlFrame() const;

    void setCurrentWrenchValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor);

    bool desactivate();

    bool activate();
    
    double getFrictionCoeff() const;

    double getMargin() const;

    void setAngleOffset(double angle_offset);

    void setFrictionCoeff(double coeff);

    void setMargin(double margin);

    void setConeOrientation(const Eigen::Matrix3d& R);

    const Eigen::Matrix3d& getConeOrientation() const;

    void setNumberOfFaces(int nfaces);

    std::shared_ptr<const common::Wrench> getWrench() const;
    
protected:
    void onResize();
    void onStart();
    void onUpdate(double current_time, double dt);
    void onStop();
private:
    std::shared_ptr<LinearizedCoulombConstraint> friction_cone_;
    std::shared_ptr<ContactExistenceConditionConstraint> ex_condition_;
    std::shared_ptr<common::Wrench> wrench_;
};

}
}
