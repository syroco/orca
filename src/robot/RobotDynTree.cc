#include "orca/robot/RobotDynTree.h"
#include "orca/utils/Utils.h"
#include "orca/math/Utils.h"
// #include "iDynTreeImpl.impl"
#include <exception>
#include <stdexcept>
#include <string>
#include <fstream>
#include <streambuf>
#include <tinyxml.h>

using namespace orca::robot;
using namespace orca::utils;
using namespace orca::math;

static bool getRobotNameFromTinyXML(TiXmlDocument* doc, std::string& model_name)
{
    TiXmlElement* robotElement = doc->FirstChildElement("robot");
    if(!robotElement)
    {
        std::cerr << "Could not get the <robot> tag in the URDF " << '\n';
        return false;
    }

    if (robotElement->Attribute("name"))
    {
        model_name = robotElement->Attribute("name");
        if(model_name.empty())
            std::cerr << "URDF has a robot name, but it is empty" << '\n';
        return true;
    }
    else
    {
        std::cerr << "Could not get the <name> tag in the URDF " << '\n';
        return false;
    }
}

RobotDynTree::RobotDynTree(const std::string& robot_name)
// : robot_impl_(new iDynTreeImpl)
: name_(robot_name)
{}

const std::string& RobotDynTree::getName() const
{
    return name_;
}

const std::vector<std::string>& RobotDynTree::getLinkNames()
{
    return link_names_;
}

const std::vector<std::string>& RobotDynTree::getFrameNames()
{
    return frame_names_;
}

const std::vector<std::string>& RobotDynTree::getJointNames()
{
    return joint_names_;
}

bool RobotDynTree::loadModelFromString(const std::string &modelString)
{
    if(name_.empty())
    {
        // If no name is provided, let's find it on the URDF
        TiXmlDocument doc;
        doc.Parse(modelString.c_str());
        if(!getRobotNameFromTinyXML(&doc,name_))
        {
            throw std::runtime_error(Formatter() << "Could not extract automatically the robot name from the urdf." \
                << '\n'
                << "Please use auto robot = std::make_shared<RobotDynTree>(\"my_robot_name\")"
                << '\n'
                );
        }
    }

    iDynTree::ModelLoader mdlLoader;
    mdlLoader.loadModelFromString(modelString);

    if(! kinDynComp_.loadRobotModel(mdlLoader.model()) )
        throw std::runtime_error(Formatter() << "Could not load model from urdf string :\n" << modelString << "\n");

    return load(kinDynComp_.model());
}

bool RobotDynTree::load(const iDynTree::Model& model)
{
    robotData_.resize(model);
    eigRobotState_.resize(model.getNrOfDOFs());
    idynRobotState_.resize(model.getNrOfDOFs());
    eigRobotState_.setFixedBaseValues();

    ndof_ = model.getNrOfDOFs();

    joint_names_.clear();
    for(unsigned int i = 0 ; i < ndof_ ; i++)
    {
        double min = 0,max = 0;
        if(kinDynComp_.model().getJoint(i)->hasPosLimits())
        {
            robotData_.eigMinJointPos[i] = kinDynComp_.model().getJoint(i)->getMinPosLimit(0);
            robotData_.eigMaxJointPos[i] = kinDynComp_.model().getJoint(i)->getMaxPosLimit(0);
        }
        else
        {
            LOG_WARNING << "Joint " << i << " does not have position limits, settings [ -inf , +inf ]";
            robotData_.eigMinJointPos[i] = - Infinity;
            robotData_.eigMaxJointPos[i] =   Infinity;
        }
        // NOTE: iDyntree stores all the joints as [actuated joints + fixed joints]
        // Important distinction then : Number of degrees of freedom != number of joints in the model !
        joint_names_.push_back( kinDynComp_.getRobotModel().getJointName(i) );
    }
    
    frame_names_.clear();
    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfFrames() ; i++)
    {
        frame_names_.push_back( kinDynComp_.getRobotModel().getFrameName(i) );
    }
    link_names_.clear();
    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfLinks() ; i++)
    {
        link_names_.push_back( kinDynComp_.getRobotModel().getLinkName(i) );
    }

    LOG_INFO << "Robot model " << getName() << " successfully loaded";

    return true;
}

bool RobotDynTree::loadModelFromFile(const std::string &modelFile)
{
    std::ifstream t(modelFile);
    std::string str((std::istreambuf_iterator<char>(t)),
                     std::istreambuf_iterator<char>());
    if( str.empty() )
        throw std::runtime_error(Formatter() << "Could not load model from urdf file \'" << modelFile << "\'");

    urdf_url_ = modelFile;
    return loadModelFromString(str);
}

const std::string& RobotDynTree::getUrdfUrl() const
{
    if(urdf_url_.empty() && !urdf_str_.empty())
        LOG_WARNING << "Robot model has been loaded with a URDF string, so the url is empty";
    assertLoaded();
    return urdf_url_;
}

const std::string& RobotDynTree::getUrdfString() const
{
    assertLoaded();
    return urdf_str_;
}

const Eigen::VectorXd& RobotDynTree::getMinJointPos()
{
    return robotData_.eigMinJointPos;
}
const Eigen::VectorXd& RobotDynTree::getMaxJointPos()
{
    return robotData_.eigMaxJointPos;
}

void RobotDynTree::print() const
{
    assertLoaded();

    std::cout << "Robot model " << getName() << '\n';
    for(unsigned int i=0; i < joint_names_.size() ; i++)
    {
        std::cout << "      Joint " << i << " " << joint_names_[i] << '\n';
    }
    for(unsigned int i=0; i < frame_names_.size() ; i++)
    {
        std::cout << "      Frame " << i << " " << frame_names_[i] << '\n';
    }
    for(unsigned int i=0; i < link_names_.size() ; i++)
    {
        std::cout << "      Link " << i << " " << link_names_[i] << '\n';
    }
}

bool RobotDynTree::isInitialized() const
{
    return is_initialized_ && ndof_ > 0;
}

void RobotDynTree::setGravity(const Eigen::Vector3d& g)
{
    global_gravity_vector_ = g;
}

void RobotDynTree::assertInitialized() const
{
    if(!is_initialized_)
        throw std::runtime_error("Robot model is not initialized with at least one state (via setRobotState)");
}

void RobotDynTree::assertLoaded() const
{
    if(ndof_ == 0)
        throw std::runtime_error("Robot model is not loaded");
}

void RobotDynTree::setBaseFrame(const std::string& base_frame)
{
    if(base_frame.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(base_frame))
        throw std::runtime_error(Formatter() << "Frame \'" << base_frame << "\' is not part of the robot");

    base_frame_ = base_frame;
    kinDynComp_.setFloatingBase( base_frame_ );
}

void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,global_gravity_vector_);
}
void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    setRobotState(Eigen::Matrix4d::Identity(),jointPos,Eigen::Matrix<double,6,1>::Zero(),jointVel,gravity);
}

void RobotDynTree::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    global_gravity_vector_ = gravity;

    assertLoaded();

    if(jointPos.size() != ndof_)
        throw std::runtime_error(Formatter() << "JointPos size do not match with current configuration : provided " << jointPos.size() << ", expected " << ndof_);

    if(jointVel.size() != ndof_)
        throw std::runtime_error(Formatter() << "JointVel size do not match with current configuration : provided " << jointVel.size() << ", expected " << ndof_);

    if(!is_initialized_)
    {
        LOG_DEBUG << "Robot is now initialized";
        is_initialized_ = true;
    }

    eigRobotState_.world_H_base = world_H_base;
    eigRobotState_.jointPos = jointPos;
    eigRobotState_.baseVel = baseVel;
    eigRobotState_.jointVel = jointVel;
    eigRobotState_.gravity = gravity;

    iDynTree::fromEigen(idynRobotState_.world_H_base,eigRobotState_.world_H_base);
    iDynTree::toEigen(idynRobotState_.jointPos) = eigRobotState_.jointPos;
    iDynTree::fromEigen(idynRobotState_.baseVel,eigRobotState_.baseVel);
    iDynTree::toEigen(idynRobotState_.jointVel) = eigRobotState_.jointVel;
    iDynTree::toEigen(idynRobotState_.gravity)  = eigRobotState_.gravity;

    kinDynComp_.setRobotState(idynRobotState_.world_H_base
                            ,idynRobotState_.jointPos
                            ,idynRobotState_.baseVel
                            ,idynRobotState_.jointVel
                            ,idynRobotState_.gravity);
}

const std::string& RobotDynTree::getBaseFrame() const
{
    if(base_frame_.empty())
        throw std::runtime_error("BaseFrame is empty. Please robot->setBaseFrame(\"some_frame_on_the_robot\")");

    assertLoaded();
    return base_frame_;
}

unsigned int RobotDynTree::getNrOfDegreesOfFreedom() const
{
    assertLoaded();
    return ndof_;
}

unsigned int RobotDynTree::getConfigurationSpaceDimension() const
{
    return 6 + ndof_;
}

const iDynTree::Model& RobotDynTree::getRobotModel()
{
    assertLoaded();
    return kinDynComp_.getRobotModel();
}

bool RobotDynTree::frameExists(const std::string& frame_name)
{
    assertLoaded();
    return kinDynComp_.getFrameIndex(frame_name) >= 0;
}

std::string RobotDynTree::getJointName(unsigned int idx)
{
    assertLoaded();
    return kinDynComp_.model().getJointName(idx);
}

unsigned int RobotDynTree::getNrOfJoints()
{
    assertLoaded();
    return kinDynComp_.model().getNrOfJoints();
}

Eigen::Matrix<double,4,4,Eigen::RowMajor> RobotDynTree::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    if(refFrameName.empty())
        throw std::runtime_error("Provided reference frame is empty");
    if(!frameExists(refFrameName))
        throw std::runtime_error(Formatter() << "Frame \'" << refFrameName << "\' is not part of the robot");

    if(frameName.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(frameName))
        throw std::runtime_error(Formatter() << "Frame \'" << frameName << "\' is not part of the robot");

    assertInitialized();
    return iDynTree::toEigen(kinDynComp_.getRelativeTransform(refFrameName,frameName).asHomogeneousTransform());
}

const Eigen::Matrix<double,6,1> RobotDynTree::getFrameVel(const std::string& frameName)
{
    if(frameName.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(frameName))
        throw std::runtime_error(Formatter() << "Frame \'" << frameName << "\' is not part of the robot");

    assertInitialized();
    return iDynTree::toEigen(kinDynComp_.getFrameVel(frameName));
}

Eigen::Map< const Eigen::Matrix<double,6,1> > RobotDynTree::getFrameBiasAcc(const std::string& frameName)
{
    if(frameName.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(frameName))
        throw std::runtime_error(Formatter() << "Frame \'" << frameName << "\' is not part of the robot");

    assertInitialized();
    return iDynTree::toEigen(kinDynComp_.getFrameBiasAcc(frameName));
}

Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > RobotDynTree::getFreeFloatingMassMatrix()
{
    assertInitialized();
    kinDynComp_.getFreeFloatingMassMatrix(robotData_.idynMassMatrix);
    return iDynTree::toEigen(robotData_.idynMassMatrix);
}

Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > RobotDynTree::getFrameFreeFloatingJacobian(const std::string& frameName)
{
    if(frameName.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(frameName))
        throw std::runtime_error(Formatter() << "Frame \'" << frameName << "\' is not part of the robot");

    assertInitialized();

    kinDynComp_.getFrameFreeFloatingJacobian(frameName,robotData_.idynJacobianFb);
    return iDynTree::toEigen(robotData_.idynJacobianFb);
}

Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > RobotDynTree::getRelativeJacobian(const std::string& refFrameName, const std::string& frameName)
{
    if(refFrameName.empty())
        throw std::runtime_error("Provided reference frame is empty");
    if(!frameExists(refFrameName))
        throw std::runtime_error(Formatter() << "Frame \'" << refFrameName << "\' is not part of the robot");

    if(frameName.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(frameName))
        throw std::runtime_error(Formatter() << "Frame \'" << frameName << "\' is not part of the robot");

    assertInitialized();
    kinDynComp_.getRelativeJacobian(kinDynComp_.getFrameIndex(refFrameName)
                                ,kinDynComp_.getFrameIndex(frameName)
                                ,robotData_.idynJacobianFb);
    return iDynTree::toEigen(robotData_.idynJacobianFb);
}

const Eigen::VectorXd& RobotDynTree::getJointPos()
{
    return eigRobotState_.jointPos;
}

const Eigen::VectorXd& RobotDynTree::getJointVel()
{
    return eigRobotState_.jointVel;
}

bool RobotDynTree::addAdditionalFrameToLink(const std::string& linkName, const std::string& frameName, const Eigen::Matrix4d& link_H_frame)
{
    if(frameName.empty())
        throw std::runtime_error("Provided frame is empty");
    if(!frameExists(frameName))
        throw std::runtime_error(Formatter() << "Frame \'" << frameName << "\' is not part of the robot");

    iDynTree::Transform idyntree_link_H_frame;
    iDynTree::fromEigen(idyntree_link_H_frame,link_H_frame);
    auto model = this->kinDynComp_.model();
    if(!model.addAdditionalFrameToLink(linkName,frameName,idyntree_link_H_frame))
        return false;
    return load(model);
}

const Eigen::VectorXd& RobotDynTree::getJointGravityTorques()
{
    assertInitialized();
    kinDynComp_.generalizedGravityForces(robotData_.generalizedGravityTorques);
    robotData_.eigJointGravityTorques = iDynTree::toEigen(robotData_.generalizedGravityTorques.jointTorques());
    return robotData_.eigJointGravityTorques;
}

const Eigen::VectorXd& RobotDynTree::getJointCoriolisTorques()
{
    generalizedBiasForces();
    getJointGravityTorques();
    robotData_.eigJointCoriolisTorques = robotData_.eigJointGravityAndCoriolisTorques - robotData_.eigJointGravityTorques;
    return robotData_.eigJointCoriolisTorques;
}

const Eigen::VectorXd& RobotDynTree::getJointGravityAndCoriolisTorques()
{
    generalizedBiasForces();
    return robotData_.eigJointGravityAndCoriolisTorques;
}

const Eigen::VectorXd& RobotDynTree::generalizedBiasForces()
{
    assertInitialized();
    kinDynComp_.generalizedBiasForces(robotData_.generalizedBiasForces);
    robotData_.eigGeneralizedBiasForces.head(6) = iDynTree::toEigen(robotData_.generalizedBiasForces.baseWrench());
    robotData_.eigJointGravityAndCoriolisTorques = iDynTree::toEigen(robotData_.generalizedBiasForces.jointTorques());
    robotData_.eigGeneralizedBiasForces.tail(ndof_) = robotData_.eigJointGravityAndCoriolisTorques;
    return robotData_.eigGeneralizedBiasForces;
}
