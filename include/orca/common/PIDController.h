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

#include <type_traits>
#include <orca/common/Mutex.h>
#include <orca/math/Utils.h>
#include <orca/util/Utils.h>

namespace orca
{
    namespace common
    {

        template<int Dimension>
        class PIDController
        {
        public:
            PIDController(MutexInterface& _mutex)
            : mutex(_mutex)
            {
                if(Dimension != Eigen::Dynamic)
                {
                    resize(Dimension);
                }
                else
                {
                    p_gain_.setZero();
                    i_gain_.setZero();
                    d_gain_.setZero();
                    i_error_.setZero();
                    d_error_.setZero();
                    cmd_.setZero();
                    windup_limit_.setConstant( math::Infinity );
                }
            }
            
            
            void resize(unsigned int dim)
            {
                MutexLock lock(mutex);
                
                if(dim > 0)
                {
                    p_gain_.setZero(dim);
                    i_gain_.setZero(dim);
                    d_gain_.setZero(dim);
                    i_error_.setZero(dim);
                    d_error_.setZero(dim);
                    cmd_.setZero(dim);
                    windup_limit_.resize(dim);
                    windup_limit_.setConstant(dim, math::Infinity );
                }
                else
                {
                    LOG_ERROR << "Dimension as to be > 0";
                }
            }

            void setProportionalGain(const Eigen::Matrix<double,Dimension,1>& P_gain)
            {
                MutexLock lock(mutex);
                
                if(P_gain.size() == p_gain_.size())
                {
                    p_gain_ = P_gain;
                }
                else
                {
                    LOG_ERROR << "Size do not match, provided " << P_gain.size() << ", but expected " << p_gain_.size();
                }
            }

            const Eigen::Matrix<double,Dimension,1>& P() const
            {
                MutexLock lock(mutex);
                
                return p_gain_;
            }

            void setIntegralGain(const Eigen::Matrix<double,Dimension,1>& I_gain)
            {
                MutexLock lock(mutex);
                
                if(I_gain.size() == i_gain_.size())
                {
                    i_gain_ = I_gain;
                }
                else
                {
                    LOG_ERROR << "Size do not match, provided " << I_gain.size() << ", but expected " << i_gain_.size();
                }
            }
            
            void setWindupLimit(const Eigen::Matrix<double,Dimension,1>& windup_lim)
            {
                MutexLock lock(mutex);
                
                windup_limit_ = windup_lim;
            }

            const Eigen::Matrix<double,Dimension,1>& WindupLimit()
            {
                MutexLock lock(mutex);
                
                return windup_limit_;
            }

            const Eigen::Matrix<double,Dimension,1>& I() const
            {
                MutexLock lock(mutex);
                
                return i_gain_;
            }
            
            void setDt(double dt)
            {
                MutexLock lock(mutex);
                
                if(dt>0)
                    dt_ = dt;
                else
                    LOG_ERROR << "dt has to be >=0";
            }
            
            void setDerivativeGain(const Eigen::Matrix<double,Dimension,1>& D_gain)
            {
                MutexLock lock(mutex);
                
                if(D_gain.size() == d_gain_.size())
                {
                    d_gain_ = D_gain;
                }
                else
                {
                    LOG_ERROR << "Size do not match, provided " << D_gain.size() << ", but expected " << d_gain_.size(); 
                }
            }

            const Eigen::Matrix<double,Dimension,1>& D() const
            {
                MutexLock lock(mutex);
                
                return d_gain_;
            }

            const Eigen::Matrix<double,Dimension,1>& computeCommand(const Eigen::Matrix<double,Dimension,1>& Error
                                                                  , const Eigen::Matrix<double,Dimension,1>& DError
            )
            {
                MutexLock lock(mutex);
                
                i_error_ += dt_ * Error;
                // saturate integral_term
                i_error_.cwiseMin(   WindupLimit() );
                i_error_.cwiseMax( - WindupLimit() );
                
                cmd_.noalias() = p_gain_.asDiagonal() * Error + i_gain_.asDiagonal() * i_error_ + d_gain_.asDiagonal() * DError;
                return cmd_;
            }

            const Eigen::Matrix<double,Dimension,1>& computeCommand(const Eigen::Matrix<double,Dimension,1>& Error)
            {
                MutexLock lock(mutex);
                
                i_error_ += dt_ * Error;
                // saturate integral_term
                i_error_.cwiseMin(   WindupLimit() );
                i_error_.cwiseMax( - WindupLimit() );
                
                d_error_ = Error / dt_;
                
                cmd_.noalias() = p_gain_.asDiagonal() * Error + i_gain_.asDiagonal() * i_error_ + d_gain_.asDiagonal() * d_error_;
                return cmd_;
            }

        private:
            Eigen::Matrix<double,Dimension,1> p_gain_;
            Eigen::Matrix<double,Dimension,1> i_gain_;
            Eigen::Matrix<double,Dimension,1> d_gain_;
            Eigen::Matrix<double,Dimension,1> windup_limit_;
            Eigen::Matrix<double,Dimension,1> i_error_;
            Eigen::Matrix<double,Dimension,1> d_error_;
            Eigen::Matrix<double,Dimension,1> cmd_;
            MutexInterface& mutex;
            double dt_ = 0;
        };
        
    } // namespace common
} // namespace orca
