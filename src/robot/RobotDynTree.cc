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

const std::vector<std::string>& RobotDynTree::getLinkNames() const
{
    return link_names_;
}

const std::vector<std::string>& RobotDynTree::getFrameNames() const
{
    return frame_names_;
}

const std::vector<std::string>& RobotDynTree::getJointNames() const
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
            orca_throw(Formatter() << "Could not extract automatically the robot name from the urdf." \
                << '\n'
                << "Please use auto robot = std::make_shared<RobotDynTree>(\"my_robot_name\")"
                << '\n'
                );
        }
    }

    iDynTree::ModelLoader mdlLoader;
    mdlLoader.loadModelFromString(modelString);

    if(! kinDynComp_.loadRobotModel(mdlLoader.model()) )
        orca_throw(Formatter() << "Could not load model from urdf string :\n" << modelString << "\n");

    return load(kinDynComp_.model());
}

bool RobotDynTree::load(const iDynTree::Model& model)
{
    robotData_.resize(model);

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
        orca_throw(Formatter() << "Could not load model from urdf file \'" << modelFile << "\'");

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
    std::cout << "  Joints" << '\n';
    for(unsigned int i=0; i < joint_names_.size() ; i++)
    {
        std::cout << "      Joint " << i << " " << joint_names_[i] << '\n';
    }
    std::cout << "  Frames" << '\n';
    for(unsigned int i=0; i < frame_names_.size() ; i++)
    {
        std::cout << "      Frame " << i << " " << frame_names_[i] << '\n';
    }
    std::cout << "  Links" << '\n';
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
    robotData_.eigRobotState.gravity = g;
}

void RobotDynTree::assertInitialized() const
{
    if(!is_initialized_)
        orca_throw("Robot model is not initialized with at least one state (via setRobotState)");
}

void RobotDynTree::assertLoaded() const
{
    if(ndof_ == 0)
        orca_throw("Robot model is not loaded");
}

void RobotDynTree::assertFrameExists(const std::string& frame) const
{
    if(frame.empty())
        orca_throw("Provided frame is empty");
    if(!frameExists(frame))
    {
        print();
        orca_throw(Formatter() << "Frame \'" << frame << "\' is not part of the robot");
    }
}

void RobotDynTree::setBaseFrame(const std::string& base_frame)
{
    assertFrameExists(base_frame);
    base_frame_ = base_frame;
    kinDynComp_.setFloatingBase( base_frame_ );
}

void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,robotData_.eigRobotState.gravity);
}
void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    setRobotState(robotData_.eigRobotState.world_H_base,jointPos,robotData_.eigRobotState.baseVel,jointVel,gravity);
}

void RobotDynTree::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    assertLoaded();

    if(jointPos.size() != ndof_)
        orca_throw(Formatter() << "JointPos size do not match with current configuration : provided " << jointPos.size() << ", expected " << ndof_);

    if(jointVel.size() != ndof_)
        orca_throw(Formatter() << "JointVel size do not match with current configuration : provided " << jointVel.size() << ", expected " << ndof_);

    if(!is_initialized_)
    {
        LOG_DEBUG << "Robot is now initialized";
        is_initialized_ = true;
    }

    robotData_.eigRobotState.world_H_base = world_H_base;
    robotData_.eigRobotState.jointPos = jointPos;
    robotData_.eigRobotState.baseVel = baseVel;
    robotData_.eigRobotState.jointVel = jointVel;
    robotData_.eigRobotState.gravity = gravity;

    robotData_.idynRobotState.fromEigen(robotData_.eigRobotState);

    kinDynComp_.setRobotState(robotData_.idynRobotState.world_H_base
                            ,robotData_.idynRobotState.jointPos
                            ,robotData_.idynRobotState.baseVel
                            ,robotData_.idynRobotState.jointVel
                            ,robotData_.idynRobotState.gravity);
}

const std::string& RobotDynTree::getBaseFrame() const
{
    if(base_frame_.empty())
        orca_throw("BaseFrame is empty. Please robot->setBaseFrame(\"some_frame_on_the_robot\")");

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

const iDynTree::Model& RobotDynTree::getRobotModel() const
{
    assertLoaded();
    return kinDynComp_.getRobotModel();
}

bool RobotDynTree::frameExists(const std::__cxx11::string& frame_name) const
{
    assertLoaded();
    return kinDynComp_.getFrameIndex(frame_name) >= 0;
}

std::string RobotDynTree::getJointName(unsigned int idx) const
{
    assertLoaded();
    return kinDynComp_.model().getJointName(idx);
}

unsigned int RobotDynTree::getNrOfJoints() const
{
    assertLoaded();
    return kinDynComp_.model().getNrOfJoints();
}

const Eigen::Matrix4d& RobotDynTree::getTransform(const std::string& frameName)
{
    return getRelativeTransform(base_frame_,frameName);
}

const Eigen::Matrix4d& RobotDynTree::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    assertFrameExists(refFrameName);
    assertFrameExists(frameName);
    assertInitialized();
    robotData_.eigTransform = iDynTree::toEigen(kinDynComp_.getRelativeTransform(refFrameName,frameName).asHomogeneousTransform());
    return robotData_.eigTransform;
}

const Eigen::Matrix<double,6,1>&  RobotDynTree::getFrameVel(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    robotData_.eigFrameVel = iDynTree::toEigen(kinDynComp_.getFrameVel(frameName));
    return robotData_.eigFrameVel;
}

const Eigen::Matrix<double,6,1>& RobotDynTree::getFrameBiasAcc(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    robotData_.eigFrameBiasAcc = iDynTree::toEigen(kinDynComp_.getFrameBiasAcc(frameName));
    return robotData_.eigFrameBiasAcc;
}

const Eigen::MatrixXd& RobotDynTree::getFreeFloatingMassMatrix()
{
    assertInitialized();

    kinDynComp_.getFreeFloatingMassMatrix(robotData_.idynFFMassMatrix);
    robotData_.eigFFMassMatrix = iDynTree::toEigen(robotData_.idynFFMassMatrix);
    return robotData_.eigFFMassMatrix;
}

const Eigen::MatrixXd& RobotDynTree::getMassMatrix()
{
    getFreeFloatingMassMatrix();
    robotData_.eigMassMatrix = robotData_.eigFFMassMatrix.block(6,6,ndof_,ndof_);
    return robotData_.eigMassMatrix;
}

const Eigen::MatrixXd& RobotDynTree::getFrameFreeFloatingJacobian(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    kinDynComp_.getFrameFreeFloatingJacobian(frameName,robotData_.idynFFJacobian);
    robotData_.eigFFJacobian = iDynTree::toEigen(robotData_.idynFFJacobian);
    return robotData_.eigFFJacobian;
}

const Eigen::MatrixXd& RobotDynTree::getJacobian(const std::string& frameName)
{
    return getRelativeJacobian(base_frame_,frameName);
}

const Eigen::MatrixXd& RobotDynTree::getRelativeJacobian(const std::string& refFrameName, const std::string& frameName)
{
    assertFrameExists(refFrameName);
    assertFrameExists(frameName);
    assertInitialized();
    kinDynComp_.getRelativeJacobian(kinDynComp_.getFrameIndex(refFrameName)
                                ,kinDynComp_.getFrameIndex(frameName)
                                ,robotData_.idynJacobian);
    robotData_.eigJacobian = iDynTree::toEigen(robotData_.idynJacobian);
    return robotData_.eigJacobian;
}

const Eigen::VectorXd& RobotDynTree::getJointPos() const
{
    return robotData_.eigRobotState.jointPos;
}

const Eigen::VectorXd& RobotDynTree::getJointVel() const
{
    return robotData_.eigRobotState.jointVel;
}

bool RobotDynTree::addAdditionalFrameToLink(const std::string& linkName, const std::string& frameName, const Eigen::Matrix4d& link_H_frame)
{
    assertFrameExists(frameName);

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
