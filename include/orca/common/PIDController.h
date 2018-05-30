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

#include "orca/common/Mutex.h"
#include "orca/math/Utils.h"
#include "orca/utils/Utils.h"

namespace orca
{
namespace common
{
    class PIDController
    {
    public:
        PIDController(unsigned int dim = 0)
        {
            if(dim > 0)
                resize(dim);
        }

        void resize(unsigned int dim)
        {
            if(dim > 0)
            {
                p_gain_.setZero(dim);
                i_gain_.setZero(dim);
                d_gain_.setZero(dim);
                i_error_.setZero(dim);
                d_error_.setZero(dim);
                cmd_.setZero(dim);
                windup_limit_.resize(dim);
                windup_limit_.setConstant( math::Infinity );
            }
            else
            {
                LOG_ERROR << "Dimension as to be > 0";
            }
        }

        void setProportionalGain(const Eigen::VectorXd& P_gain)
        {
            utils::assertSize(P_gain,p_gain_);
            p_gain_ = P_gain;
        }

        const Eigen::VectorXd& P() const
        {
            return p_gain_;
        }

        void setIntegralGain(const Eigen::VectorXd& I_gain)
        {
            utils::assertSize(I_gain,i_gain_);
            i_gain_ = I_gain;
        }

        void setWindupLimit(const Eigen::VectorXd& windup_lim)
        {
            utils::assertSize(windup_lim,windup_limit_);
            windup_limit_ = windup_lim;
        }

        const Eigen::VectorXd& windupLimit()
        {
            return windup_limit_;
        }

        const Eigen::VectorXd& I() const
        {
            return i_gain_;
        }

        void setDerivativeGain(const Eigen::VectorXd& D_gain)
        {
            utils::assertSize(D_gain,d_gain_);
            d_gain_ = D_gain;
        }

        const Eigen::VectorXd& D() const
        {
            return d_gain_;
        }

        const Eigen::VectorXd& computeCommand(const Eigen::VectorXd& Error
                                          , const Eigen::VectorXd& DError
                                          , double dt)
        {
            i_error_ += dt * Error;
            // saturate integral_term
            i_error_.cwiseMin(   windupLimit() );
            i_error_.cwiseMax( - windupLimit() );

            cmd_.noalias() = p_gain_.asDiagonal() * Error + i_gain_.asDiagonal() * i_error_ + d_gain_.asDiagonal() * DError;
            return cmd_;
        }

        const Eigen::VectorXd& computeCommand(const Eigen::VectorXd& Error, double dt)
        {
            i_error_ += dt * Error;
            // saturate integral_term
            i_error_.cwiseMin(   windupLimit() );
            i_error_.cwiseMax( - windupLimit() );

            d_error_ = Error / dt;

            cmd_.noalias() = p_gain_.asDiagonal() * Error + i_gain_.asDiagonal() * i_error_ + d_gain_.asDiagonal() * d_error_;
            return cmd_;
        }

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
