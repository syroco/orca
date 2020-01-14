/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#include <kdl_codyco/utils.hpp>

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/config.h>

#include <iostream>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <sstream>
#include <fstream>
#include <Eigen/LU>

namespace KDL {
namespace CoDyCo {



    double computeMass(const Tree & tree) {

        double total_mass = 0.0;

        //create necessary vectors
        SegmentMap::const_iterator root;

        root = tree.getRootSegment();

        SegmentMap sm = tree.getSegments();

        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            //root has no mass
            if( i != root ) {
               total_mass += GetTreeElementSegment(i->second).getInertia().getMass();
            }
        }

        return total_mass;
    }


    int JointInvertPolarity(const KDL::Joint & old_joint, const KDL::Frame & old_f_tip, KDL::Joint & new_joint, KDL::Frame & new_f_tip)
    {

        switch(old_joint.getType())
        {
            case Joint::RotAxis:
            case Joint::RotX:
            case Joint::RotY:
            case Joint::RotZ:
                //\todo document this
                new_f_tip = KDL::Frame(old_f_tip.M.Inverse(),-(old_f_tip.M.Inverse()*old_joint.JointOrigin()));
                new_joint = Joint(old_joint.getName(),-(old_f_tip.M.Inverse()*old_f_tip.p), -(old_f_tip.M.Inverse()*old_joint.JointAxis()), Joint::RotAxis);
            break;
            case Joint::TransAxis:
            case Joint::TransX:
            case Joint::TransY:
            case Joint::TransZ:
                new_f_tip = KDL::Frame(old_f_tip.M.Inverse(),-(old_f_tip.M.Inverse()*old_f_tip.p));
                new_joint = Joint(old_joint.getName(),-(old_f_tip.M.Inverse()*old_joint.JointOrigin()), -(old_f_tip.M.Inverse()*old_joint.JointAxis()), Joint::TransAxis);
            break;
            case Joint::None:
            default:
                new_f_tip = KDL::Frame(old_f_tip.M.Inverse(),-(old_f_tip.M.Inverse()*old_f_tip.p));
                new_joint = Joint(old_joint.getName(), Joint::None);
            break;
        }
        #ifndef NDEBUG
        //std::cerr << "Exiting joint polarity inversion, checking all is working" << std::endl;
        //double q = 0.83;
        //std::cerr << old_joint.pose(q)*old_f_tip*new_joint.pose(q)*new_f_tip << std::endl;
        #endif
        return 0;
    }

    Twist operator/(const Wrench& h, const RigidBodyInertia& I)
    {
        //For mathematical details, check:
        //Featherstone Book (RBDA, 2008) formuma 2.74, page 36
        double m = I.getMass();
        Vector com_kdl = I.getCOG();
        RotationalInertia Irot = I.getRotationalInertia();

        if( m == 0 ) { return Twist::Zero(); }

        Eigen::Matrix3d Ic, Ic_inverse, skew_com;
        Eigen::Vector3d com;

        com = Eigen::Map<Eigen::Vector3d>(com_kdl.data);
        skew_com = skew(com);
        Ic = Eigen::Map<Eigen::Matrix3d>(Irot.data) + m*skew_com*skew_com;
        Ic_inverse = Ic.inverse();


        Twist return_value;
        Eigen::Map<Eigen::Vector3d>(return_value.vel.data) =  ((1/m)*Eigen::Matrix3d::Identity() - skew_com*Ic_inverse*skew_com) * Eigen::Map<const Eigen::Vector3d>(h.force.data) +
                                                              skew_com*Ic_inverse*Eigen::Map<const Eigen::Vector3d>(h.torque.data);

        Eigen::Map<Eigen::Vector3d>(return_value.rot.data) =  (-Ic_inverse*skew_com)*Eigen::Map<const Eigen::Vector3d>(h.force.data)
                                                              + Ic_inverse*Eigen::Map<const Eigen::Vector3d>(h.torque.data);

        return return_value;
    }

    bool multiplyInertiaJacobian(const Jacobian& src, const RigidBodyInertia& I, MomentumJacobian& dest)
    {
        if(src.columns()!=dest.columns())
            return false;
        for(unsigned int i=0;i<src.columns();i++)
            dest.setColumn(i,I*src.getColumn(i));;
        return true;
    }

    bool divideJacobianInertia(const MomentumJacobian& src, const RigidBodyInertia& I, Jacobian& dest)
    {
        /** \todo if the inertia matrix is singular ? */
        if(src.columns()!=dest.columns() || I.getMass() == 0)
            return false;
        for(unsigned int i=0;i<src.columns();i++)
            dest.setColumn(i,src.getColumn(i)/I);
        return true;
    }

    bool stringVectorFromFile(const std::string filename, std::vector<std::string> & strings, int nr_of_string_to_read)
    {
        std::ifstream links_file;

        links_file.open (filename.c_str(), std::ifstream::in);

        if( !links_file ) {
            std::cerr << "KDL::CoDyCo::stringVectorFromFile error: could not load file " << filename << std::endl;
            return false;
        }

        std::string data_buffer;

        if( nr_of_string_to_read > 0 ) {
            if( (int)strings.size() != nr_of_string_to_read ) { strings.resize(nr_of_string_to_read); }

            for(int i=0; i < nr_of_string_to_read; i++ ) {
                getline(links_file,data_buffer);
                if( data_buffer == "" ) {
                    std::cerr << "KDL::CoDyCo::stringVectorFromFile error: file " << filename << " is not properly formatted at line " << i << std::endl;
                    return false;
                }
                strings[i] = data_buffer;
            }
        } else {
            strings.resize(0);
            while(getline(links_file,data_buffer) ) {
                if( data_buffer == "" ) {
                    std::cerr << "KDL::CoDyCo::stringVectorFromFile error: file " << filename << " is not properly formatted at line " << strings.size()+1 << std::endl;
                    return false;
                }
                strings.push_back(data_buffer);
            }
        }

        return true;
    }

    bool isBaseLinkFake(const KDL::Tree & tree)
    {
        KDL::SegmentMap::const_iterator root = tree.getRootSegment();
        bool return_value;
        if (GetTreeElementChildren(root->second).size() == 1 && GetTreeElementSegment(GetTreeElementChildren(root->second)[0]->second).getJoint().getType() == Joint::None) {
            return_value = true;
        } else {
            return_value = false;
        }
        return return_value;
    }

    void spatialToConventionalAcceleration(const KDL::Twist spatial_acc,
                                           const KDL::Twist velocity,
                                                 KDL::Twist & conventional_acc)
    {
         conventional_acc.rot = spatial_acc.rot;
         conventional_acc.vel = spatial_acc.vel-velocity.vel*velocity.rot;
    }

    void conventionalToSpatialAcceleration(const KDL::Twist conventional_acc,
                                           const KDL::Twist velocity,
                                                 KDL::Twist & spatial_acc)
    {
         spatial_acc.rot = conventional_acc.rot;
         spatial_acc.vel = conventional_acc.vel+velocity.vel*velocity.rot;
    }


}
}
