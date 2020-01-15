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

#include <kdl_codyco/momentumjacobian.hpp>

namespace KDL
{
namespace CoDyCo
{

    using namespace Eigen;

    MomentumJacobian::MomentumJacobian()
    {
    }


    MomentumJacobian::MomentumJacobian(unsigned int nr_of_columns):
        data(6,nr_of_columns)
    {
    }

    MomentumJacobian::MomentumJacobian(const MomentumJacobian& arg):
        data(arg.data)
    {
    }

    MomentumJacobian& MomentumJacobian::operator = (const MomentumJacobian& arg)
    {
        this->data=arg.data;
        return *this;
    }


    MomentumJacobian::~MomentumJacobian()
    {

    }

    void MomentumJacobian::resize(unsigned int new_nr_of_columns)
    {
        data.resize(6,new_nr_of_columns);
    }

    double MomentumJacobian::operator()(unsigned int i,unsigned int j)const
    {
        return data(i,j);
    }

    double& MomentumJacobian::operator()(unsigned int i,unsigned int j)
    {
        return data(i,j);
    }

    unsigned int MomentumJacobian::rows()const
    {
        return data.rows();
    }

    unsigned int MomentumJacobian::columns()const
    {
        return data.cols();
    }

    void SetToZero(MomentumJacobian& jac)
    {
        jac.data.setZero();
    }

    void MomentumJacobian::changeRefPoint(const Vector& base_AB){
        for(unsigned int i=0;i<(unsigned int)data.cols();i++)
            this->setColumn(i,this->getColumn(i).RefPoint(base_AB));
    }

    bool changeRefPoint(const MomentumJacobian& src1, const Vector& base_AB, MomentumJacobian& dest)
    {
        if(src1.columns()!=dest.columns())
            return false;
        for(unsigned int i=0;i<src1.columns();i++)
            dest.setColumn(i,src1.getColumn(i).RefPoint(base_AB));
        return true;
    }

    void MomentumJacobian::changeBase(const Rotation& rot){
        for(unsigned int i=0;i<(unsigned int)data.cols();i++)
            this->setColumn(i,rot*this->getColumn(i));;
    }

    bool changeBase(const MomentumJacobian& src1, const Rotation& rot, MomentumJacobian& dest)
    {
        if(src1.columns()!=dest.columns())
            return false;
        for(unsigned int i=0;i<src1.columns();i++)
            dest.setColumn(i,rot*src1.getColumn(i));;
        return true;
    }

    void MomentumJacobian::changeRefFrame(const Frame& frame){
        for(unsigned int i=0;i<(unsigned int)data.cols();i++)
            this->setColumn(i,frame*this->getColumn(i));
    }

    bool changeRefFrame(const MomentumJacobian& src1,const Frame& frame, MomentumJacobian& dest)
    {
        if(src1.columns()!=dest.columns())
            return false;
        for(unsigned int i=0;i<src1.columns();i++)
            dest.setColumn(i,frame*src1.getColumn(i));
        return true;
    }

    bool MomentumJacobian::operator ==(const MomentumJacobian& arg)const
    {
        return Equal((*this),arg);
    }

    bool MomentumJacobian::operator!=(const MomentumJacobian& arg)const
    {
        return !Equal((*this),arg);
    }

    bool Equal(const MomentumJacobian& a,const MomentumJacobian& b,double eps)
    {
        if(a.rows()==b.rows()&&a.columns()==b.columns()){
            return a.data.isApprox(b.data,eps);
        }else
            return false;
    }

    Wrench MomentumJacobian::getColumn(unsigned int i) const{
        return Wrench(Vector(data(0,i),data(1,i),data(2,i)),Vector(data(3,i),data(4,i),data(5,i)));
    }

    void MomentumJacobian::setColumn(unsigned int i,const Wrench& t){
        data.col(i).head<3>()=Eigen::Map<const Vector3d>(t.force.data);
        data.col(i).tail<3>()=Eigen::Map<const Vector3d>(t.torque.data);
    }
}
}
