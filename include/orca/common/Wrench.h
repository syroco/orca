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

#include "orca/robot/RobotModel.h"

namespace orca
{
namespace common
{

class Wrench
{
public:
    Wrench(const std::string& name);

    void setRobotModel(std::shared_ptr<robot::RobotModel> robot);

    virtual ~Wrench();

    const std::string& getName() const;

    bool isActivated() const;

    void activate();

    void deactivate();

    void setBaseFrame(const std::string& base_ref_frame);

    void setControlFrame(const std::string& control_frame);

    void setCurrentValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor);

    const Eigen::Matrix<double,6,1>& getCurrentValue() const;

    const std::string& getBaseFrame() const;

    const std::string& getControlFrame() const;

    const Eigen::MatrixXd& getJacobianTranspose() const;

    const Eigen::MatrixXd& getJacobian() const;

    virtual void print() const;

    virtual void update(double current_time, double dt);
protected:
    virtual void resize();
private:
    std::string base_ref_frame_,control_frame_;
    Eigen::MatrixXd jacobian_transpose_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jac_zero_;
    Eigen::Matrix<double,6,1> current_wrench_;
    Eigen::Matrix<double,6,1> wrench_zero_;
    std::shared_ptr<robot::RobotModel> robot_;
    bool is_activated_ = true;
    const std::string name_;
};

}
}
