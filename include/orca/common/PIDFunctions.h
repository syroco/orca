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
#include <Eigen/Dense>
#include <orca/math/Utils.h>

namespace orca
{
    namespace common
    {

        template<int Dimension>
        class PIDFunctions
        {
        public:
            void setDimension(unsigned int dim)
            {
                P_gain_.setZero(dim);
                I_gain_.setZero(dim);
                D_gain_.setZero(dim);
                Windup_limit_.resize(dim);
                Windup_limit_.setConstant(dim, math::Infinity );
                return;
            }

            void setProportionalGain(const Eigen::Matrix<double,Dimension,1>& P_gain)
            {
                if(P_gain.size() == P_gain_.size())
                {
                    P_gain_ = P_gain;
                }
                else
                {

                }
                return;
            }

            const Eigen::Matrix<double,Dimension,1>& P() const
            {
                return P_gain_;
            }

            void setIntegralGain(const Eigen::Matrix<double,Dimension,1>& I_gain)
            {
                if(I_gain.size() == I_gain_.size())
                {
                    I_gain_ = I_gain;
                }
                else
                {

                }
                return;
            }

            void setWindupLimit(const Eigen::Matrix<double,Dimension,1>& windup_lim)
            {
                Windup_limit_ = windup_lim;
                return;
            }

            const Eigen::Matrix<double,Dimension,1>& WindupLimit()
            {
                return Windup_limit_;
            }

            const Eigen::Matrix<double,Dimension,1>& I() const
            {
                return I_gain_;
            }

            void setDerivativeGain(const Eigen::Matrix<double,Dimension,1>& D_gain)
            {
                if(D_gain.size() == D_gain_.size())
                {
                    D_gain_ = D_gain;
                }
                else
                {

                }
                return;
            }

            const Eigen::Matrix<double,Dimension,1>& D() const
            {
                return D_gain_;
            }

//             const Eigen::Matrix<double,Dimension,1>& computeCommand(const Eigen::Matrix<double,Dimension,1>& Error
//                                                                     , const Eigen::Matrix<double,Dimension,1>& IError
//                                                                     , const Eigen::Matrix<double,Dimension,1>& DError
//             )
//             {
//                 return P_gain_.asDiagonal() * Error + I_gain_.asDiagonal() * IError + D_gain_.asDiagonal() * DError;
//             }

        private:
            Eigen::Matrix<double,Dimension,1> P_gain_;
            Eigen::Matrix<double,Dimension,1> I_gain_;
            Eigen::Matrix<double,Dimension,1> D_gain_;
            Eigen::Matrix<double,Dimension,1> Windup_limit_;
            //Eigen::Matrix<double,Dimension,1> error_, _;
        };
    }
}
