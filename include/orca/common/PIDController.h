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

#include "orca/math/Utils.h"
#include "orca/utils/Utils.h"

namespace orca
{
namespace common
{
    class PIDController
    {
    public:
        PIDController(unsigned int dim = 0);
        void resize(unsigned int dim);
        void setProportionalGain(const Eigen::VectorXd& P_gain);
        const Eigen::VectorXd& P() const;
        void setIntegralGain(const Eigen::VectorXd& I_gain);
        void setWindupLimit(const Eigen::VectorXd& windup_lim);
        const Eigen::VectorXd& windupLimit();
        const Eigen::VectorXd& I() const;
        void setDerivativeGain(const Eigen::VectorXd& D_gain);

        const Eigen::VectorXd& D() const;
        const Eigen::VectorXd& computeCommand(const Eigen::VectorXd& Error
                                          , const Eigen::VectorXd& DError
                                          , double dt);
        const Eigen::VectorXd& computeCommand(const Eigen::VectorXd& Error, double dt);
        const void print() const;
    private:
        Eigen::VectorXd p_gain_;
        Eigen::VectorXd i_gain_;
        Eigen::VectorXd d_gain_;
        Eigen::VectorXd windup_limit_;
        Eigen::VectorXd i_error_;
        Eigen::VectorXd d_error_;
        Eigen::VectorXd cmd_;
    };

} // namespace common
} // namespace orca
