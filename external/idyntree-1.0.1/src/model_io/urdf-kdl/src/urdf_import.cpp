/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Wim Meeussen */

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_compatibility.h>
#include <fstream>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <tinyxml.h>

using namespace KDL;

namespace iDynTree{


void printTree(urdf::ConstLinkPtr link,int level = 0)
{
  level+=2;
  int count = 0;
  for (urdf::LinkVector::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
    }
  }

}

// construct vector
Vector toKdl(urdf::Vector3 v)
{
  return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation toKdl(urdf::Rotation r)
{
  return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame toKdl(urdf::Pose p)
{
  return Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
Joint toKdl(urdf::JointPtr jnt)
{
  Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

  switch (jnt->type){
  case urdf::Joint::FIXED:{
    return Joint(jnt->name, Joint::None);
  }
  case urdf::Joint::REVOLUTE:{
    Vector axis = toKdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
  }
  case urdf::Joint::CONTINUOUS:{
    Vector axis = toKdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
  }
  case urdf::Joint::PRISMATIC:{
    Vector axis = toKdl(jnt->axis);
    return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::TransAxis);
  }
  default:{
    std::cerr << "Converting unknown joint type of joint " << jnt->name << " into a fixed joint" << std::endl;
    return Joint(jnt->name, Joint::None);
  }
  }
  return Joint();
}

// construct inertia
RigidBodyInertia toKdl(urdf::InertialPtr i)
{
  Frame origin = toKdl(i->origin);

  // the mass is frame indipendent
  double kdl_mass = i->mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  Vector kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  RotationalInertia urdf_inertia =
    RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  RigidBodyInertia kdl_inertia_wrt_com_workaround =
    origin.M *RigidBodyInertia(0, Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  RotationalInertia kdl_inertia_wrt_com =
    kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return RigidBodyInertia(kdl_mass,kdl_com,kdl_inertia_wrt_com);
}


// recursive function to walk through tree
bool addChildrenToTree(urdf::LinkPtr root, Tree& tree)
{
  urdf::LinkVector children = root->child_links;
  //std::cerr << "[INFO] Link " << root->name << " had " << children.size() << " children" << std::endl;

  // constructs the optional inertia
  RigidBodyInertia inert(0);
  if (root->inertial)
    inert = toKdl(root->inertial);

  // constructs the kdl joint
  Joint jnt = toKdl(root->parent_joint);

  // construct the kdl segment
  Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);

  // recurslively add all children
  for (size_t i=0; i<children.size(); i++){
    if (!addChildrenToTree(children[i], tree))
      return false;
  }
  return true;
}


bool treeFromUrdfFile(const std::string& file, Tree& tree,const bool consider_root_link_inertia)
{
    std::ifstream ifs(file.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return treeFromUrdfString(xml_string,tree,consider_root_link_inertia);
}

/*
bool treeFromParam(const string& param, Tree& tree)
{
  urdf::ModelInterface robot_model;
  if (!robot_model.initParam(param)){
    logError("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
}
*/


int print_tree(urdf::ModelInterfacePtr & robot) {
std::cout << "robot name is: " << robot->getName() << std::endl;

  // get info from parser
  std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
  // get root link
  urdf::ConstLinkPtr root_link=robot->getRoot();
  if (!root_link) return -1;

  std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;



  // print entire tree
  printTree(root_link);

  return 0;
}

bool treeFromUrdfString(const std::string& xml, Tree& tree, const bool consider_root_link_inertia)
{
  urdf::ModelInterfacePtr urdf_model;
  urdf_model = urdf::parseURDF(xml);
  if( !urdf_model )
  {
      std::cerr << "[ERR] Could not parse string to urdf::ModelInterface" << std::endl;
      return false;
  }
  return treeFromUrdfModel(*urdf_model,tree,consider_root_link_inertia);
}

/*
bool treeFromUrdfXml(TiXmlDocument *xml_doc, Tree& tree)
{
  urdf::ModelInterfacePtr robot_model;
  robot_model = urdf::parseURDF(
  if (!robot_model.parse(xml_doc)){
    logError("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
}*/


bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree, const bool consider_root_link_inertia)
{
  if (consider_root_link_inertia) {
    //For giving a name to the root of KDL using the robot name,
    //as it is not used elsewhere in the KDL tree
    std::string fake_root_name = "__kdl_import__" + robot_model.getName()+"__fake_root__";
    std::string fake_root_fixed_joint_name = "__kdl_import__" + robot_model.getName()+"__fake_root_fixed_joint__";

    tree = Tree(fake_root_name);

   const urdf::ConstLinkPtr root = robot_model.getRoot();

    // constructs the optional inertia
    RigidBodyInertia inert(0);
    if (root->inertial)
      inert = toKdl(root->inertial);

    // constructs the kdl joint
    Joint jnt = Joint(fake_root_fixed_joint_name, Joint::None);

    // construct the kdl segment
    Segment sgm(root->name, jnt, Frame::Identity(), inert);

    // add segment to tree
    tree.addSegment(sgm, fake_root_name);

  } else {
    tree = Tree(robot_model.getRoot()->name);

    // warn if root link has inertia. KDL does not support this
    if (robot_model.getRoot()->inertial)
      std::cerr << "The root link " << robot_model.getRoot()->name <<
                   " has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF." << std::endl;
  }

  //  add all children
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
    if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree))
      return false;

  return true;
}


bool jointPosLimitsFromUrdfFile(const std::string& file,
                             std::vector<std::string> & joint_names,
                             KDL::JntArray & min,
                             KDL::JntArray & max)
{
    std::ifstream ifs(file.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return jointPosLimitsFromUrdfString(xml_string,joint_names,min,max);
}


bool jointPosLimitsFromUrdfString(const std::string& urdf_xml,
                               std::vector<std::string> & joint_names,
                               KDL::JntArray & min,
                               KDL::JntArray & max)
{
  urdf::ModelInterfacePtr urdf_model;
  urdf_model = urdf::parseURDF(urdf_xml);
  if( !urdf_model )
  {
      std::cerr << "[ERR] Could not parse string to urdf::ModelInterface" << std::endl;
      return false;
  }
  return jointPosLimitsFromUrdfModel(*urdf_model,joint_names,min,max);
}

bool jointPosLimitsFromUrdfModel(const urdf::ModelInterface& robot_model,
                              std::vector<std::string> & joint_names,
                              KDL::JntArray & min,
                              KDL::JntArray & max)
{
    int nrOfJointsWithLimits=0;
    for (urdf::JointPtrMap::const_iterator it=robot_model.joints_.begin(); it!=robot_model.joints_.end(); ++it)
    {
        if( it->second->type == urdf::Joint::REVOLUTE ||
            it->second->type == urdf::Joint::PRISMATIC )
        {
            nrOfJointsWithLimits++;
        }
    }

    joint_names.resize(nrOfJointsWithLimits);
    min.resize(nrOfJointsWithLimits);
    max.resize(nrOfJointsWithLimits);

    int index =0;
    for (urdf::JointPtrMap::const_iterator it=robot_model.joints_.begin(); it!=robot_model.joints_.end(); ++it)
    {
        if( it->second->type == urdf::Joint::REVOLUTE ||
            it->second->type == urdf::Joint::PRISMATIC )
        {
            joint_names[index] = (it->first);
            min(index) = it->second->limits->lower;
            max(index) = it->second->limits->upper;
            index++;
        }
    }

    if( index != nrOfJointsWithLimits )
    {
        std::cerr << "[ERR] kdl_format_io error in jointPosLimitsFromUrdfModel function" << std::endl;
        return false;
    }

    return true;
}

bool framesFromKDLTree(const KDL::Tree& tree,
                       std::vector<std::string>& framesNames,
                       std::vector<std::string>& parentLinkNames)
{
    framesNames.clear();
    parentLinkNames.clear();

    KDL::SegmentMap::iterator seg;
    KDL::SegmentMap segs;
    KDL::SegmentMap::const_iterator root_seg;
    root_seg = tree.getRootSegment();
    segs = tree.getSegments();
    for( seg = segs.begin(); seg != segs.end(); seg++ )
    {
        if( GetTreeElementChildren(seg->second).size() == 0 &&
            GetTreeElementSegment(seg->second).getJoint().getType() == KDL::Joint::None &&
            GetTreeElementSegment(seg->second).getInertia().getMass() == 0.0 )
        {
            std::string frameName = GetTreeElementSegment(seg->second).getName();
            std::string parentLinkName = GetTreeElementSegment(GetTreeElementParent(seg->second)->second).getName();
            framesNames.push_back(frameName);
            parentLinkNames.push_back(parentLinkName);

            //also check parent
            KDL::Segment parent = GetTreeElementSegment(GetTreeElementParent(seg->second)->second);
            if (parent.getJoint().getType() == KDL::Joint::None &&
                parent.getInertia().getMass() == 0.0)
            {
                framesNames.push_back(parentLinkName);
            }
        }
    }

    return true;
}


}

