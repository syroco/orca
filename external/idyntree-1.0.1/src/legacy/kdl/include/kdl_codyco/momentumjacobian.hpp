// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_CODYCO_MOMENTUM_JACOBIAN_HPP
#define KDL_CODYCO_MOMENTUM_JACOBIAN_HPP

#ifdef __DEPRECATED
  #warning <momentumjacobian.hpp> is deprecated.
#endif

#include <kdl/frames.hpp>
#include <Eigen/Core>

namespace KDL
{
namespace CoDyCo
{
    class MomentumJacobian;
    // Equal is friend function, but default arguments for friends are forbidden (§8.3.6.4)
    bool Equal(const MomentumJacobian& a,const MomentumJacobian& b,double eps=epsilon);
    
    /**
     * This is basically a copy of the KDL::Jacobian class, the only difference is that 
     * its columns are Wrenches, while in ordinary Jacobians the columns are Twists. This is useful 
     * for expressing mapping between joint velocities and elements of M^6, for example the spatial
     * momentum.
     * 
     */
    class MomentumJacobian
    {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<double,6,Eigen::Dynamic> data;
        MomentumJacobian();
        explicit MomentumJacobian(unsigned int nr_of_columns);
        MomentumJacobian(const MomentumJacobian& arg);

        ///Allocates memory for new size (can break realtime behavior)
        void resize(unsigned int newNrOfColumns);

        ///Allocates memory if size of this and argument is different
        MomentumJacobian& operator=(const MomentumJacobian& arg);

        bool operator ==(const MomentumJacobian& arg)const;
        bool operator !=(const MomentumJacobian& arg)const;
        
        friend bool Equal(const MomentumJacobian& a,const MomentumJacobian& b,double eps);
        

        ~MomentumJacobian();

        double operator()(unsigned int i,unsigned int j)const;
        double& operator()(unsigned int i,unsigned int j);
        unsigned int rows()const;
        unsigned int columns()const;

        friend void SetToZero(MomentumJacobian& jac);

        friend bool changeRefPoint(const MomentumJacobian& src1, const Vector& base_AB, MomentumJacobian& dest);
        friend bool changeBase(const MomentumJacobian& src1, const Rotation& rot, MomentumJacobian& dest);
        friend bool changeRefFrame(const MomentumJacobian& src1,const Frame& frame, MomentumJacobian& dest);

        Wrench getColumn(unsigned int i) const;
        void setColumn(unsigned int i,const Wrench& t);

        void changeRefPoint(const Vector& base_AB);
        void changeBase(const Rotation& rot);
        void changeRefFrame(const Frame& frame);


    };

    bool changeRefPoint(const MomentumJacobian& src1, const Vector& base_AB, MomentumJacobian& dest);
    bool changeBase(const MomentumJacobian& src1, const Rotation& rot, MomentumJacobian& dest);
    bool changeRefFrame(const MomentumJacobian& src1,const Frame& frame, MomentumJacobian& dest);

}
}

#endif
