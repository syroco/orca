/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#include <iDynTree/Core/Wrench.h>

#include "iCub/iDynTree/TorqueEstimationTree.h"
#include "iCub/iDynTree/idyn2kdl_icub.h"

//Urdf import from kdl_format_io
#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>

#include "iDynTree/Sensors/Sensors.h"
#include "iDynTree/Sensors/SixAxisForceTorqueSensor.h"
#include <iDynTree/Core/Transform.h>

#include <kdl_codyco/rnea_loops.hpp>

#include "kdl_codyco/KDLConversions.h"
#include <kdl_codyco/position_loops.hpp>

#include <yarp/math/SVD.h>
#include <yarp/os/Log.h>

#include <vector>
#include <iCub/iDynTree/yarp_kdl.h>

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/regressors/dirl_utils.hpp>

using namespace yarp::sig;
using namespace yarp::math;

namespace iCub {
namespace iDynTree {


TorqueEstimationTree::TorqueEstimationTree(std::string urdf_filename,
                                           std::vector<std::string> dof_serialization,
                                           std::vector<std::string> ft_serialization,
                                           std::string fixed_link,
                                           unsigned int verbose)
{
    yarp::sig::Vector q_min_yarp, q_max_yarp;

    //Parse a KDL::Tree from URDF
    KDL::Tree icub_kdl;

    bool ret = ::iDynTree::treeFromUrdfFile(urdf_filename,icub_kdl);

    assert(ret);
    if( !ret ) {
        { std::cerr << "[INFO] TorqueEstimationTree: error in costructor" << std::endl; }
        return;
    }

    //Construct F/T sensor name list from URDF gazebo extensions
    std::vector< ::iDynTree::FTSensorData> ft_sensors;
    ret = ::iDynTree::ftSensorsFromUrdfFile(urdf_filename, ft_sensors);

    if( !ret )
    {
        {
            std::cerr << "[ERR] TorqueEstimationTree: error in loading ft_sensors" << std::endl;
            assert(false);
        }
        return;
    }


    int nrOfDofs = dof_serialization.size();
    KDL::JntArray q_min_kdl(nrOfDofs), q_max_kdl(nrOfDofs);
    std::vector<std::string> joint_limits_names;
    ::iDynTree::jointPosLimitsFromUrdfFile(urdf_filename,joint_limits_names,q_min_kdl,q_max_kdl);


    //Set joint limits
    q_min_yarp.resize(nrOfDofs);
    q_max_yarp.resize(nrOfDofs);

    for(int dof = 0; dof < nrOfDofs; dof++ )
    {
        std::string dof_name = dof_serialization[dof];
        for(size_t lim = 0; lim < joint_limits_names.size(); lim++ )
        {
            if( joint_limits_names[lim] == dof_name )
            {
                q_min_yarp[dof] = q_min_kdl(lim);
                q_max_yarp[dof] = q_max_yarp(lim);
                break;
            }
        }
    }

    this->TorqueEstimationConstructor(icub_kdl,ft_sensors,
                                      dof_serialization,ft_serialization,
                                      q_min_yarp,q_max_yarp,fixed_link,verbose);



}

TorqueEstimationTree::TorqueEstimationTree(KDL::Tree& icub_kdl,
                                          std::vector< ::iDynTree::FTSensorData > ft_sensors,
                                          std::vector< std::string > dof_serialization,
                                          std::vector< std::string > ft_serialization,
                                          yarp::sig::Vector& q_min, yarp::sig::Vector& q_max,
                                          std::string fixed_link, unsigned int verbose)
{
    TorqueEstimationConstructor(icub_kdl,ft_sensors,dof_serialization,ft_serialization,q_min,q_max,fixed_link,verbose);
}

TorqueEstimationTree::TorqueEstimationTree(KDL::Tree& icub_kdl,
                                          std::vector< ::iDynTree::FTSensorData > ft_sensors,
                                          std::vector< std::string > ft_serialization,
                                          std::string fixed_link, unsigned int verbose)
{
    yarp::sig::Vector q_max(icub_kdl.getNrOfJoints(),1000.0);
    yarp::sig::Vector q_min(icub_kdl.getNrOfJoints(),-1000.0);

    KDL::CoDyCo::TreeSerialization serial(icub_kdl);

    std::vector<std::string> dof_serialization;

    for(size_t i = 0; i < serial.getNrOfDOFs(); i++ )
    {
        dof_serialization.push_back(serial.getDOFName(i));
    }

    TorqueEstimationConstructor(icub_kdl,ft_sensors,dof_serialization,ft_serialization,q_min,q_max,fixed_link,verbose);
}



void TorqueEstimationTree::TorqueEstimationConstructor(KDL::Tree & icub_kdl,
                                                  std::vector< ::iDynTree::FTSensorData> ft_sensors,
                                                  std::vector<std::string> dof_serialization,
                                                  std::vector<std::string> ft_serialization,
                                                  yarp::sig::Vector & q_min_yarp, yarp::sig::Vector & q_max_yarp,
                                                  std::string fixed_link, unsigned int verbose)
{
    std::vector< std::string > ft_names(ft_serialization.size());
    std::vector<KDL::Frame> child_sensor_transforms(ft_serialization.size());
    KDL::Frame kdlFrame;

    for(std::size_t serialization_id=0; serialization_id < ft_serialization.size(); serialization_id++)
    {
        if( 0 == ft_sensors.size() )
        {
                std::cerr << "[ERR] TorqueEstimationTree: ft sensor " << ft_serialization[serialization_id] << " not found in model file." << std::endl;
                assert(false);
                return;
        }
        for(std::size_t ft_sens=0; ft_sens < ft_sensors.size(); ft_sens++ )
        {
            std::string ft_sens_name = ft_sensors[ft_sens].reference_joint;
            std::size_t ft_sens_id;


            if( ft_serialization[serialization_id] == ft_sens_name)
            {
                ft_sens_id = serialization_id;

                ft_names[ft_sens_id] = ft_sens_name;
                // \todo TODO FIXME properly address also parent and child cases
                //                  and measure_direction
                if( ft_sensors[ft_sens].frame == ::iDynTree::FTSensorData::SENSOR_FRAME )
                {
                    child_sensor_transforms[ft_sens_id] = KDL::Frame(ft_sensors[ft_sens].sensor_pose.M);
                }
                else
                {
                    child_sensor_transforms[ft_sens_id] = KDL::Frame::Identity();
                }

                break;
            }

            if( ft_sens == ft_sensors.size() -1 )
            {
                std::cerr << "[ERR] TorqueEstimationTree: ft sensor " << ft_sens_name << " not found in model file." << std::endl;
                assert(false);
                return;
            }
        }
    }

        std::cerr << "[INFO] TorqueEstimationTree constructor: loaded urdf with " << dof_serialization.size()
              << "dofs and " << ft_names.size() << " fts ( " << ft_serialization.size() <<  ") " << std::endl;

    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

    //Setting a custom dof serialization (\todo TODO FIXME : quite an hack, substitute with proper)
    if( dof_serialization.size() != 0 )
    {
        yAssert(dof_serialization.size() == serial.getNrOfDOFs());
        for(std::size_t dof=0; dof < dof_serialization.size(); dof++)
        {
            std::string dof_string = dof_serialization[dof];
            yAssert(serial.getDOFID(dof_string) != -1);
            yAssert(serial.getJunctionID(dof_string) != -1);
        }

        for(std::size_t dof=0; dof < dof_serialization.size(); dof++)
        {
            std::string dof_string = dof_serialization[dof];
            std::cout << "[DEBUG] TorqueEstimationTree: Setting id of dof " << dof_string << " to " << dof << std::endl;
            serial.setDOFNameID(dof_string, (int)dof);
            serial.setJunctionNameID(dof_string, (int)dof);
        }
    }

    std::string imu_link_name = "imu_frame";

    if( fixed_link != "" )
    {
        imu_link_name = fixed_link;
    }

    this->constructor(icub_kdl,ft_names,imu_link_name,serial);

    std::cerr << "[INFO] TorqueEstimationTree constructor: loaded urdf with " << this->getNrOfDOFs()
              << "dofs and " << ft_names.size() << " fts ( " << ft_serialization.size() <<  ") " << std::endl;

    assert(this->getNrOfDOFs() > 0);



    this->setJointBoundMin(q_min_yarp);
    this->setJointBoundMax(q_max_yarp);

    //iDynTreeContact
    int ret = buildSubGraphStructure(ft_names);
    if( ret != 0 ) { std::cerr << "iDynTree constructor: ft sensor specified not found" << std::endl; }

    //building matrix and vectors for each subgraph
    contacts.resize(NrOfDynamicSubGraphs);
    A_contacts.resize(NrOfDynamicSubGraphs);
    b_contacts.resize(NrOfDynamicSubGraphs,Vector(6,0.0));
    x_contacts.resize(NrOfDynamicSubGraphs);

    b_contacts_subtree.resize(NrOfLinks);

    //end iDynTreeContact

    return;
}

//====================================
//
//      iDynTreeContact methods
//
//====================================

bool TorqueEstimationTree::isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const
{
    if (std::find(ft_sensors.begin(), ft_sensors.end(), joint_name) != ft_sensors.end())
    {
        return true;
    }
    //else
    {
        return false;
    }
}

bool TorqueEstimationTree::generateSensorsTree(const std::vector<std::string> & ft_names,
                                    const std::vector<bool> & is_measure_direction_child_to_parent)
{
    for(size_t i=0; i < ft_names.size(); i++ )
    {
        //Creating a new ft sensor to be added in the ft sensors structure
        ::iDynTree::SixAxisForceTorqueSensor new_sens;


        if( this->undirected_tree.getJunction(ft_names[i]) != this->undirected_tree.getInvalidJunctionIterator() )
        {
            //Set the sensor name (for the time being equal to the junction name)
            new_sens.setName(ft_names[i]);
            //Set the junction name
            new_sens.setParentJoint(ft_names[i]);
            int junction_index = this->undirected_tree.getJunction(ft_names[i])->getJunctionIndex();
            new_sens.setParentJointIndex(junction_index);
            KDL::CoDyCo::JunctionMap::const_iterator junct_it = this->undirected_tree.getJunction(ft_names[i]);

            int parent_index = junct_it->getParentLink()->getLinkIndex();
            int child_index = junct_it->getChildLink()->getLinkIndex();
            std::string parent_link_name = junct_it->getParentLink()->getName();
            std::string child_link_name = junct_it->getChildLink()->getName();

            if( is_measure_direction_child_to_parent[i] )
            {
                new_sens.setAppliedWrenchLink(parent_index);
            }
            else
            {
                new_sens.setAppliedWrenchLink(child_index);
            }

            // Currently we support only the case where the ft sensor frame is equal
            // to the child link frame
            new_sens.setSecondLinkSensorTransform(child_index,::iDynTree::Transform::Identity());
            new_sens.setSecondLinkName(child_link_name);

            // Then, the parent_link_H_sensor transform is simply parent_link_H_child_link transform
            KDL::Frame parent_link_H_sensor = junct_it->pose(0.0,false);
            new_sens.setFirstLinkSensorTransform(parent_index,::iDynTree::ToiDynTree(parent_link_H_sensor));
            new_sens.setFirstLinkName(parent_link_name);

        }
        else
        {
            std::cerr << "[ERR] DynTree::generateSensorsTree: problem generating sensor for ft "
                      << ft_names[i] << std::endl;
            return false;
        }

        int ret = sensors_tree.addSensor(new_sens);

        assert(ret == i);
    }

    return true;
}


int TorqueEstimationTree::buildSubGraphStructure(const std::vector<std::string> & ft_names)
{
    link2subgraph_index.resize(NrOfLinks,-1);
    link_is_subgraph_root.resize(NrOfLinks,false);
    subgraph_index2root_link.resize(NrOfDynamicSubGraphs,-1);

    int next_id = 0;

    for(int i=0; i < dynamic_traversal.getNrOfVisitedLinks(); i++) {

        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(i);
        int link_nmbr = link_it->getLinkIndex();

        if( i == 0 ) {

            //Starting with the dynamical base link, assign an index to the subgraph
            assert( dynamic_traversal.getParentLink(link_nmbr) == undirected_tree.getInvalidLinkIterator() );
            link2subgraph_index[link_nmbr] = next_id;

            //The dynamical base link is the root of its subgraph
            link_is_subgraph_root[link_nmbr] = true;
            subgraph_index2root_link[link2subgraph_index[link_nmbr]] = link_nmbr;

            next_id++;

        } else {
            //For every link, the subgraph is the same of its parent, unless it is connected to it by an FT sensor
            KDL::CoDyCo::LinkMap::const_iterator parent_it = dynamic_traversal.getParentLink(link_it->getLinkIndex());
            int parent_nmbr = parent_it->getLinkIndex();

            if( isFTsensor(link_it->getAdjacentJoint(parent_it)->getJoint().getName(),ft_names) ) {
                //The FT sensor should be a fixed joint ? probably not
                //assert(link_it->getAdjacentJoint(parent_it)->joint.getType() == Joint::None);

                link2subgraph_index[link_nmbr] = next_id;

                //This link is a root of a dynamical subgraph, as its parent is in another subgraph
                link_is_subgraph_root[link_nmbr] = true;
                subgraph_index2root_link[link2subgraph_index[link_nmbr]] = link_nmbr;

                next_id++;
            } else {
                link2subgraph_index[link_nmbr] = link2subgraph_index[parent_nmbr];

                //This link is not a root of a dynamical subgraph
                link_is_subgraph_root[link_nmbr] = false;
            }
        }
    }

    //Building Force/Torque sensors data structures
    std::vector<bool> is_measure_direction_child_to_parent(ft_names.size(),true);
    //ft_list = KDL::CoDyCo::FTSensorList(undirected_tree,ft_names,is_measure_direction_child_to_parent);
    this->generateSensorsTree(ft_names,is_measure_direction_child_to_parent);

    //The numbers of ids must be equal to the number of subgraphs
    if(next_id == (int)NrOfDynamicSubGraphs) {
        return 0;
    } else {
        assert(false);
        return -1;
    }
}


std::vector<yarp::sig::Vector> TorqueEstimationTree::getSubTreeInternalDynamics()
{
    computePositions();
    std::vector<yarp::sig::Vector> return_value(NrOfDynamicSubGraphs,Vector(6,0.0));

    std::vector<KDL::Wrench> return_value_kdl(NrOfDynamicSubGraphs,KDL::Wrench::Zero());

    for(int i=0; i < dynamic_traversal.getNrOfVisitedLinks(); i++ ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(i);
        return_value_kdl[link2subgraph_index[link_it->getLinkIndex()]] += X_dynamic_base[link_it->getLinkIndex()]*f_gi[link_it->getLinkIndex()];
    }

    for(size_t i=0; i < NrOfDynamicSubGraphs; i++ ) {
        for(size_t j=0; j < 6; j++ ) {
            return_value[i][j] = return_value_kdl[i](j);
        }
    }

    return return_value;
}

bool TorqueEstimationTree::addSkinDynLibAlias(const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                                const int skinDynLib_body_part, const int skinDynLib_link_index)
{
   int iDynTree_link_index = this->getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERR] addSkinDynLibAlias : link " << iDynTree_link_name << " not found in the model " << std::endl;
   }

   int iDynTree_frame_index = this->getLinkIndex(iDynTree_frame_name);
   if( iDynTree_frame_index < 0 )
   {
       std::cerr << "[ERR] addSkinDynLibAlias : frame " << iDynTree_frame_name << " not found in the model " << std::endl;
   }

   skinDynLibLinkID sdl_id;
   sdl_id.body_part = skinDynLib_body_part;
   sdl_id.local_link_index = skinDynLib_link_index;

   iDynTreeLinkAndFrame idyntree_id;
   idyntree_id.link_index = iDynTree_link_index;
   idyntree_id.frame_index = iDynTree_frame_index;

   //Remove any existing alias for this link to avoid anomalies
   this->removeSkinDynLibAlias(iDynTree_link_name);

   skinDynLibLinkMap.insert(std::pair<skinDynLibLinkID,iDynTreeLinkAndFrame>(sdl_id,idyntree_id));

   return true;
}

bool TorqueEstimationTree::getSkinDynLibAlias(const std::string iDynTree_link_name, std::string & iDynTree_frame_name,
                                int & skinDynLib_body_part, int & skinDynLib_link_index)
{
   int iDynTree_link_index = this->getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERR] getSkinDynLibAlias : link " << iDynTree_link_name << " not found " << std::endl;
       return false;
   }

     // TODO \todo What is this crazyness??? Linear search of a map??
   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == iDynTree_link_index )
       {
           skinDynLib_body_part = it->first.body_part;
           skinDynLib_link_index = it->first.local_link_index;
           int iDynTree_frame_index = it->second.frame_index;
           iDynTree_frame_name = undirected_tree.getLink(iDynTree_frame_index)->getName();
           break;
       }
   }

   return true;

}

//FIXME TODO \todo implemente this method with an appropriate data structure, such that
//  it has a complexity of O(1)
bool TorqueEstimationTree::getSkinDynLibAlias(const int iDynTree_link_index, int & iDynTree_frame_index,
                        int & skinDynLib_body_part, int & skinDynLib_link_index)
{
  if( iDynTree_link_index < 0 || iDynTree_link_index >= this->getNrOfLinks() ) return false;

  // TODO \todo What is this crazyness??? Linear search of a map??
   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == iDynTree_link_index )
       {
           skinDynLib_body_part = it->first.body_part;
           skinDynLib_link_index = it->first.local_link_index;
           iDynTree_frame_index = it->second.frame_index;
           return true;
       }
   }

   return false;

}

bool TorqueEstimationTree::removeSkinDynLibAlias(std::string link)
{
   int link_index = this->getLinkIndex(link);
   if( link_index < 0 )
   {
       std::cerr << "[ERR] removeSkinDynLibAlias : link " << link << " not found " << std::endl;
   }

   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == link_index )
       {
           skinDynLibLinkMap.erase(it);
           break;
       }
   }

   return true;
}

bool TorqueEstimationTree::skinDynLib2iDynTree(const int skinDynLib_body_part,
                                 const int skinDynLib_link_index,
                                 int & iDynTree_link_index,
                                 int & iDynTree_frame_index)
{
    skinDynLibLinkID skinID;
    skinID.body_part = skinDynLib_body_part;
    skinID.local_link_index = skinDynLib_link_index;

    std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.find(skinID);

    if( it == skinDynLibLinkMap.end() )
    {
        std::cerr << "[ERR] DynTree::skinDynLib2iDynTree : skinDynLib link "
                  << skinDynLib_body_part << " " << skinDynLib_link_index << " not found " << std::endl;
        return false;
    }

    iDynTree_link_index  = it->second.link_index;
    iDynTree_frame_index = it->second.frame_index;

    return true;
}


bool TorqueEstimationTree::setContacts(const iCub::skinDynLib::dynContactList & contacts_list)
{
    assert((int)contacts.size() == NrOfDynamicSubGraphs);
    for(size_t sg = 0; sg < NrOfDynamicSubGraphs; sg++ ) {
        contacts[sg].resize(0);
    }

    //Separate unknown contacts depending on their subgraph
    for(iCub::skinDynLib::dynContactList::const_iterator it = contacts_list.begin();
            it != contacts_list.end(); it++ )
    {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }


        int subgraph_id = getSubGraphIndex(iDynTree_link_index);

        contacts[subgraph_id].push_back(*it);
    }

    are_contact_estimated = false;

    return true;
}

const iCub::skinDynLib::dynContactList TorqueEstimationTree::getContacts() const
{
    iCub::skinDynLib::dynContactList all_contacts(0);


    for(size_t sg = 0; sg < NrOfDynamicSubGraphs; sg++ )
    {
        all_contacts.insert(all_contacts.end(),contacts[sg].begin(),contacts[sg].end());
    }

    return all_contacts;
}


TorqueEstimationTree::~TorqueEstimationTree() {}


void TorqueEstimationTree::buildAb_contacts()
{
//    #ifndef NDEBUG
//    bool extreme_verbose = false;
//    #endif
    //First calculate the known terms b related to inertial, gravitational and
    //measured F/T

    for(int l=dynamic_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(l);
            int link_nmbr = link_it->getLinkIndex();
            //Collect RigidBodyInertia and external forces
            KDL::RigidBodyInertia Ii= link_it->getInertia();
            //This calculation should be done one time in forward kineamtic loop and stored \todo
            b_contacts_subtree[link_nmbr] = Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr]) - getMeasuredWrench(link_nmbr);
            #ifndef NDEBUG
            /*
            if(extreme_verbose) {
            std::cerr << "link_nmbr : " << link_nmbr << std::endl;
            std::cerr << "b_contacts_subtree: " << b_contacts_subtree[link_nmbr] << std::endl;
            std::cerr << "a " << a[link_nmbr] << std::endl;
            std::cerr << "v " << v[link_nmbr] << std::endl;
            std::cerr << "getMeasuredWrench " << getMeasuredWrench(link_nmbr) << std::endl;
            }
            */
            #endif
    }


    for(int l=dynamic_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {

        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(l);
        int link_nmbr = link_it->getLinkIndex();

        if( l != 0 ) {

            KDL::CoDyCo::LinkMap::const_iterator parent_it = dynamic_traversal.getParentLink(link_nmbr);
            const int parent_nmbr = parent_it->getLinkIndex();
            //If this link is a subgraph root, store the result, otherwise project it to the parent
            #ifndef NDEBUG
            //std::cerr << "Link_nmbr " << link_nmbr << std::endl;
            //std::cerr << "isSubGraphRoot(" << link_nmbr << ") " << isSubGraphRoot(link_nmbr) << std::endl;
            #endif
            if( !isSubGraphRoot(link_nmbr) ) {
                double joint_pos;

                KDL::CoDyCo::JunctionMap::const_iterator joint_it = link_it->getAdjacentJoint(parent_it);
                if( joint_it->getJoint().getType() == KDL::Joint::None ) {
                    joint_pos = 0.0;
                } else {
                    joint_pos = q(link_it->getAdjacentJoint(parent_it)->getDOFIndex());
                }
                b_contacts_subtree[parent_nmbr] += link_it->pose(parent_it,joint_pos)*b_contacts_subtree[link_nmbr];
            }
       }

       if( isSubGraphRoot(link_nmbr ) )
       {
           //std::cout << " setting b_contact [ " << getSubGraphIndex(link_nmbr) << " ] ";
           //std::cout << "to " << b_contacts_subtree[link_nmbr] << std::endl;
           b_contacts[getSubGraphIndex(link_nmbr)].setSubvector(0,KDLtoYarp(b_contacts_subtree[link_nmbr].force));
           b_contacts[getSubGraphIndex(link_nmbr)].setSubvector(3,KDLtoYarp(b_contacts_subtree[link_nmbr].torque));
           //std::cout << " b_contacts [ " << getSubGraphIndex(link_nmbr) << " ] is " << b_contacts[getSubGraphIndex(link_nmbr)].toString() << std::endl;
       }


    }

    //Then calculate the A and b related to unknown contacts

    iCub::skinDynLib::dynContactList::const_iterator it;

    std::vector<int> unknowns(NrOfDynamicSubGraphs,0);

    //Calculate the number of unknowns for each subgraph
    for(size_t sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        for(it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }

            if(it->isMomentKnown())
            {
                if(it->isForceDirectionKnown())
                {
                    unknowns[sg]++;     // 1 unknown (force module)
                }
                else
                {
                    unknowns[sg]+=3;    // 3 unknowns (force)
                }
            }
            else
            {
                unknowns[sg]+=6;        // 6 unknowns (force and moment)
            }
        }
    }


    for(size_t sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        //Resize the A matrices
        A_contacts[sg] = yarp::sig::Matrix(6,unknowns[sg]);
        A_contacts[sg].zero();

        //Calculate A and b related to contacts
        int colInd = 0;
        for( it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }

            //Subgraph information
            int subgraph_index = link2subgraph_index[iDynTree_link_index];

            assert(subgraph_index == sg);

            // We will write the force estimation minimization problem in
            // the plucker frame of the root link of the subgraph
            int subgraph_root = subgraph_index2root_link[subgraph_index];

            //Get Frame transform between contact link and subgraph root
            //Inefficient but leads to cleaner code, if necessary can be improved
            KDL::Frame H_root_link = getFrameLoop(undirected_tree,q,dynamic_traversal,subgraph_root,iDynTree_link_index);
            // \todo TODO this transformation is constant, just store it somewhere
            KDL::Frame H_link_skinFrame = getFrameLoop(undirected_tree,q,dynamic_traversal,iDynTree_link_index,iDynTree_skinFrame_index);

            // skinDynLib express the force in a frame that can be
            // different from iDynTree link frame, we will call this
            // skinFrame . The COP got from the the skinContactList is
            // expressed in this skinFrame, and we will estimate the force
            // and the torque in this skinFrame because the force will be
            // still streamed out in a skinContactList form.
            KDL::Vector COP_skinFrame;
            YarptoKDL(it->getCoP(),COP_skinFrame);

            // An additional frame is the frame oriented as the skinFrame, but
            // with the origin in the contact COP . We will call this frame the
            // "contactFrame"

            KDL::Frame H_skinFrame_contactFrame = KDL::Frame(COP_skinFrame);
            KDL::Frame H_root_contactFrame = H_root_link*H_link_skinFrame*H_skinFrame_contactFrame;

            if(it->isForceDirectionKnown())
            {
                // 1 UNKNOWN: FORCE MODULE
                yarp::sig::Matrix un(6,1);
                un.zero();
                un.setSubcol(it->getForceDirection(),0,0); // force direction unit vector
                yarp::sig::Matrix H_adj_root_contact = KDLtoYarp_wrench(H_root_contactFrame);
                yarp::sig::Matrix col = H_adj_root_contact*un;
                A_contacts[sg].setSubmatrix(col,0,colInd);
                colInd += 1;

            }
            else
            {
                if( it->isMomentKnown() ) {
                    // 3 UNKNOWNS: FORCE
                    A_contacts[sg].setSubmatrix(KDLtoYarp_wrench(H_root_contactFrame).submatrix(0,5,0,2),0,colInd);
                    colInd += 3;

                } else {
                    // 6 UNKNOWNS: FORCE AND MOMENT
                    A_contacts[sg].setSubmatrix(KDLtoYarp_wrench(H_root_contactFrame),0,colInd);
                    colInd += 6;
                }

            }
        }
    }


}

void TorqueEstimationTree::store_contacts_results()
{
    //Make sure that the external forces are equal to zero before storing the results
    for(int l=dynamic_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
        f_ext[dynamic_traversal.getOrderedLink(l)->getLinkIndex()] = KDL::Wrench::Zero();
    }

    for(size_t sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        unsigned int unknownInd = 0;
        iCub::skinDynLib::dynContactList::iterator it;
        for(it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }

            //Store the result in dynContactList, for output
            if(it->isForceDirectionKnown()) {
                //1 UNKNOWN
                it->setForceModule( x_contacts[sg](unknownInd++));
            }
            else
            {
                if(it->isMomentKnown())
                {
                    //3 UNKNOWN
                    assert( unknownInd+2 < x_contacts[sg].size() );
                    it->setForce(x_contacts[sg].subVector(unknownInd, unknownInd+2));
                    unknownInd += 3;
                } else {
                    //6 UNKNOWN
                    assert( unknownInd+2 < x_contacts[sg].size() );
                    it->setForce(x_contacts[sg].subVector(unknownInd, unknownInd+2));
                    unknownInd += 3;
                    assert( unknownInd+2 < x_contacts[sg].size() );
                    it->setMoment(x_contacts[sg].subVector(unknownInd, unknownInd+2));
                    unknownInd += 3;

                }
            }

            //Store the results in f_ext, for RNEA dynamic loop
            // f_ext is expressed in the link Plucker frame,
            // while skinContactList wrenches are expressed in the contactFrame
            KDL::Vector COP, force, moment;
            YarptoKDL(it->getCoP(),COP);
            KDL::Frame H_skinFrame_contactFrame = KDL::Frame(COP);
            YarptoKDL(it->getForce(),force);
            YarptoKDL(it->getMoment(),moment);

            KDL::Wrench f_ext_contactFrame = KDL::Wrench(force,moment);

            // Todo store and avoid repeated computations
            KDL::Frame H_link_skinFrame = getFrameLoop(undirected_tree,q,dynamic_traversal,iDynTree_link_index,iDynTree_skinFrame_index);

            KDL::Frame H_link_contactFrame = H_link_skinFrame*H_skinFrame_contactFrame;

            f_ext[iDynTree_link_index] = f_ext[iDynTree_link_index] + H_link_contactFrame*f_ext_contactFrame;
        }
    }
}

bool TorqueEstimationTree::estimateContactForcesFromSkin()
{
    #ifndef NDEBUG
    //std::cout << "DynTree::estimateContactForces " << std::endl;
    #endif

    double tol = 1e-7; /**< value extracted from old iDynContact */
    buildAb_contacts();
    for(size_t i=0; i < NrOfDynamicSubGraphs; i++ ) {
        #ifndef NDEBUG
        /*
        std::cout << "A_contacts " << i << " has size " << A_contacts[i].rows() << " " << A_contacts[i].cols() << std::endl;
        std::cout << A_contacts[i].toString() << std::endl;
        std::cout << "b_contacts " << i << " has size " << b_contacts[i].size() << std::endl;
        std::cout << b_contacts[i].toString() << std::endl;
        */
        #endif
        x_contacts[i] = yarp::math::pinv(A_contacts[i],tol)*b_contacts[i];
        #ifndef NDEBUG

        /*
        std::string contacts_string = x_contacts[i].toString();

        std::cout << "x_contacts " << i << " has size " << x_contacts[i].size() << std::endl;
        std::cout << x_contacts[i].toString() << std::endl;

        std::cout << "A_contacts*x_contacts : " << std::endl;
        std::cout << (A_contacts[i]*x_contacts[i]).toString() << std::endl;
        */

        #endif
    }
    store_contacts_results();
    are_contact_estimated = true;
    return true;
}


KDL::Wrench TorqueEstimationTree::getMeasuredWrench(int link_id)
{

    ::iDynTree::Wrench total_measured_applied_wrench = ::iDynTree::Wrench::Zero();
    for(size_t ft=0; ft < NrOfFTSensors; ft++ )
    {
        ::iDynTree::SixAxisForceTorqueSensor * sens
            = (::iDynTree::SixAxisForceTorqueSensor *) sensors_tree.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE,ft);

        assert(sens != 0);

        ::iDynTree::Wrench measured_wrench_on_link = ::iDynTree::Wrench();//::iDynTree::ToiDynTree(KDL::Wrench::Zero());
        ::iDynTree::Wrench measured_wrench_by_sensor;

        bool ok = sensor_measures.getMeasurement(::iDynTree::SIX_AXIS_FORCE_TORQUE,ft,measured_wrench_by_sensor);

        assert(ok);

        // If the sensor with index ft is not attached to the link
        // this function return a zero wrench
        sens->getWrenchAppliedOnLink(link_id,measured_wrench_by_sensor,measured_wrench_on_link);

        //Sum the given wrench to the return value
        total_measured_applied_wrench = total_measured_applied_wrench+measured_wrench_on_link;
    }

    return ::iDynTree::ToKDL(total_measured_applied_wrench);
}

bool TorqueEstimationTree::dynamicRNEA()
{
    int ret;
    //ret = rneaDynamicLoop(undirected_tree,q,dynamic_traversal,v,a,f_ext,f,torques,base_residual_f);
    ret = KDL::CoDyCo::rneaDynamicLoop(undirected_tree,q,dynamic_traversal,f_gi,f_ext,f,torques,base_residual_f);
    //Check base force: if estimate contact was called, it should be zero
    if( are_contact_estimated == true )
    {
        //If the force were estimated wright
        #ifndef NDEBUG
        /*
        std::cout << "q:   " << q.data << std::endl;
        std::cout << "dq:  " << dq.data << std::endl;
        std::cout << "ddq: " << ddq.data << std::endl;
        for(int i=0; i < f_ext.size(); i++ ) { std::cout << "f_ext[" << i << "]: " << f_ext[i] << std::endl; }
        */
        //std::cerr << "base_residual_f.force.Norm " << base_residual_f.force.Norm() << std::endl;
        //std::cerr << "base_residual_f.force.Norm " << base_residual_f.torque.Norm() << std::endl;

        #endif
        if(  base_residual_f.force.Norm() > 1e-5 )
        {
            std::cout << "iDynTree WARNING: base_residual_f.force.Norm() is " << base_residual_f.force.Norm() << " instead of zero." << std::endl;
            ret = -1;
        }
        if(  base_residual_f.torque.Norm() > 1e-5 )
        {
            std::cout << "iDynTree WARNING: base_residual_f.torque.Norm() is " << base_residual_f.torque.Norm() << " instead of zero." << std::endl;
            ret = -1;
        }
        //Note: this (that no residual appears happens only for the proper selection of the provided dynContactList
    }
    else
    {
        //In case contacts forces where not estimated, the sensor values have
        //to be calculated from the RNEA
        for(size_t i=0; i < NrOfFTSensors; i++ )
        {
            //Todo add case that the force/wrench is the one of the parent ?
            ::iDynTree::Wrench measure_wrench;

            ::iDynTree::SixAxisForceTorqueSensor * p_ft_sensor =
                reinterpret_cast< ::iDynTree::SixAxisForceTorqueSensor *>(sensors_tree.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE,i));

            bool ok = KDL::CoDyCo::Regressors::simulateMeasurement_sixAxisFTSensor(dynamic_traversal,f,p_ft_sensor,measure_wrench);

            assert(ok);

            ok = sensor_measures.setMeasurement(::iDynTree::SIX_AXIS_FORCE_TORQUE,i,measure_wrench);

            assert(ok);
        }
    }
    return ret >= 0;
}


int TorqueEstimationTree::getFTSensorIndex(const std::string & ft_name) const
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(ft_name);

    if( junction_it == undirected_tree.getInvalidJunctionIterator() )
    {
        std::cerr << "DynTree::getFTSensorIndex : junction " << ft_name << " not found" << std::endl;
        return -1;
    }

    if( junction_it->getNrOfDOFs() > 0 )
    {
        std::cerr << "DynTree::getFTSensorIndex warning: " << ft_name << " is not a fixed junction " << std::endl;
    }

    unsigned int junction_index = junction_it->getJunctionIndex();
    // Search the ft index given the associated junction index
    assert( sensors_tree.getNrOfSensors(::iDynTree::SIX_AXIS_FORCE_TORQUE) == NrOfFTSensors );
    for( size_t ft = 0; ft < NrOfFTSensors; ft++ )
    {
        ::iDynTree::SixAxisForceTorqueSensor * p_ft_sensor =
                dynamic_cast< ::iDynTree::SixAxisForceTorqueSensor *>(sensors_tree.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE,ft));
        assert(p_ft_sensor != 0);
        int ft_junction_index = p_ft_sensor->getParentJointIndex();
        if( ft_junction_index == junction_index )
        {
            return ft;
        }
    }

    return -1;

    //return ft_list.getFTSensorID(junction_it->getJunctionIndex());
}

}
}
