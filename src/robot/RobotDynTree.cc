#include "orca/robot/RobotDynTree.h"
#include "orca/utils/Utils.h"
#include "orca/math/Utils.h"
#include "RobotDynTree_iDynTree.impl"
#include <exception>
#include <stdexcept>
#include <string>
#include <fstream>
#include <streambuf>
#include <tinyxml.h>

using namespace orca::robot;
using namespace orca::utils;
using namespace orca::math;

#define assertInitialized() \
    if(!is_initialized_) \
        orca_throw("Robot model is not initialized with at least one state (via setRobotState)");

#define assertLoaded() \
    if(impl_->getNrOfDegreesOfFreedom() == 0) \
        orca_throw("Robot model is not loaded");

#define assertFrameExists(frame) \
    if(frame.empty()) \
        orca_throw("Provided frame is empty"); \
    if(!impl_->frameExists(frame)) { \
        print(); \
        orca_throw(Formatter() << "Frame \'" << frame << "\' is not part of the robot"); \
    }

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
: name_(robot_name)
{
    switch(robot_kinematics_type_)
    {
        case RobotDynTreeType::iDynTree:
            impl_ = make_unique<RobotDynTreeImpl<iDynTree> >();
            break;
        default:
            orca_throw(Formatter() << "RobotDynTree only support iDynTree for now");
    }
}

RobotDynTree::~RobotDynTree()
{

}

const std::string& RobotDynTree::getName() const
{
    return name_;
}

const std::vector<std::string>& RobotDynTree::getLinkNames() const
{
    return impl_->getLinkNames();
}

const std::vector<std::string>& RobotDynTree::getFrameNames() const
{
    return impl_->getFrameNames();
}

const std::vector<std::string>& RobotDynTree::getJointNames() const
{
    return impl_->getJointNames();
}

bool RobotDynTree::loadModelFromString(const std::string &modelString)
{
    // Extract the model name from the URDF
    // WARNING : in multi robot environnement + ROS
    // This will cause topic names collisions as they are based on robot names
    if(name_.empty())
    {
        // If no name is provided, let's find it on the URDF
        TiXmlDocument doc;
        doc.Parse(modelString.c_str());
        if(!getRobotNameFromTinyXML(&doc,name_))
        {
            std::cerr << "modelString : \n" << modelString << '\n';
            LOG_ERROR << "Could not extract automatically the robot name from the urdf." \
                << '\n'
                << "Please use auto robot = std::make_shared<RobotDynTree>(\"my_robot_name\")";
        }
        else
        {
            LOG_DEBUG << "Name extracted from URDF string : " << name_;
        }
    }
    if(impl_->loadModelFromString(modelString))
    {
        urdf_str_ = modelString;
        LOG_INFO << "Robot model " << getName() << " successfully loaded";
        return true;
    }
    std::cerr << "modelString : \n" << modelString << "\n\n";
    orca_throw(Formatter() << "Could not load robot model from string");
    return false;
}

bool RobotDynTree::loadModelFromFile(const std::string &modelFile)
{
    std::ifstream t(modelFile);
    std::string str((std::istreambuf_iterator<char>(t)),
                     std::istreambuf_iterator<char>());
    if( str.empty() )
        LOG_ERROR << "Could not load model from urdf file \'" << modelFile << "\' because file is empty";

    if(loadModelFromString(str))
    {
        urdf_url_ = modelFile;
        return true;
    }
    return false;
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
    if(urdf_str_.empty())
        LOG_ERROR << "Robot is not loaded, URDF string is empty";
    return urdf_str_;
}

const Eigen::VectorXd& RobotDynTree::getMinJointPos()
{
    return impl_->getMinJointPos();
}
const Eigen::VectorXd& RobotDynTree::getMaxJointPos()
{
    return impl_->getMaxJointPos();
}

void RobotDynTree::print() const
{
    assertLoaded();

    auto jn = getJointNames();
    auto fn = getFrameNames();
    auto ln = getLinkNames();

    std::cout << "Robot model " << getName() << '\n';
    std::cout << "  Joints" << '\n';
    for(unsigned int i=0; i < jn.size() ; i++)
    {
        std::cout << "      Joint " << i << " " << jn[i] << '\n';
    }
    std::cout << "  Frames" << '\n';
    for(unsigned int i=0; i < fn.size() ; i++)
    {
        std::cout << "      Frame " << i << " " << fn[i] << '\n';
    }
    std::cout << "  Links" << '\n';
    for(unsigned int i=0; i < ln.size() ; i++)
    {
        std::cout << "      Link " << i << " " << ln[i] << '\n';
    }
}

void RobotDynTree::onRobotInitializedCallback(std::function<void(void)> cb)
{
    robot_initialized_cb_ = cb;
}

bool RobotDynTree::isInitialized() const
{
    return is_initialized_ && getNrOfDegreesOfFreedom() > 0;
}

void RobotDynTree::setGravity(const Eigen::Vector3d& g)
{
    impl_->setGravity(g);
}

void RobotDynTree::setBaseFrame(const std::string& base_frame)
{
    assertFrameExists(base_frame);
    return impl_->setBaseFrame( base_frame );
}

void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,impl_->getGravity());
}
void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    setRobotState(impl_->getWorldToBaseTransform(),jointPos,impl_->getBaseVelocity(),jointVel,gravity);
}

void RobotDynTree::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    assertLoaded();

    if(jointPos.size() != getNrOfDegreesOfFreedom())
        orca_throw(Formatter() << "JointPos size do not match the current configuration : provided " << jointPos.size() << ", expected " << getNrOfDegreesOfFreedom());

    if(jointVel.size() != getNrOfDegreesOfFreedom())
        orca_throw(Formatter() << "JointVel size do not match the current configuration : provided " << jointVel.size() << ", expected " << getNrOfDegreesOfFreedom());

    impl_->setRobotState(world_H_base
                        ,jointPos
                        ,baseVel
                        ,jointVel
                        ,gravity);
    if(!is_initialized_)
    {
        LOG_DEBUG << "Robot " << getName() << "is now initialized";
        is_initialized_ = true;
        if(robot_initialized_cb_)
            robot_initialized_cb_();
    }
}

const std::string& RobotDynTree::getBaseFrame() const
{
    assertLoaded();

    if(impl_->getBaseFrame().empty())
        orca_throw("BaseFrame is empty. Please robot->setBaseFrame(\"some_frame_on_the_robot_that_attach_him_to_the_ground\")");

    return impl_->getBaseFrame();
}

unsigned int RobotDynTree::getNrOfDegreesOfFreedom() const
{
    assertLoaded();
    return impl_->getNrOfDegreesOfFreedom();
}

unsigned int RobotDynTree::getConfigurationSpaceDimension() const
{
    return 6 + getNrOfDegreesOfFreedom();
}

bool RobotDynTree::frameExists(const std::string& frame_name) const
{
    assertLoaded();
    return impl_->frameExists(frame_name);
}

std::string RobotDynTree::getJointName(unsigned int idx) const
{
    assertLoaded();
    return impl_->getJointName(idx);
}

unsigned int RobotDynTree::getNrOfJoints() const
{
    assertLoaded();
    return impl_->getNrOfJoints();
}

const Eigen::Matrix4d& RobotDynTree::getTransform(const std::string& frameName)
{
    return getRelativeTransform(impl_->getBaseFrame(),frameName);
}

const Eigen::Matrix4d& RobotDynTree::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    assertFrameExists(refFrameName);
    assertFrameExists(frameName);
    assertInitialized();
    return impl_->getRelativeTransform(refFrameName,frameName);
}

const Eigen::Matrix<double,6,1>&  RobotDynTree::getFrameVel(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getFrameVel(frameName);
}

const Eigen::Matrix<double,6,1>& RobotDynTree::getFrameBiasAcc(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getFrameBiasAcc(frameName);
}

const Eigen::MatrixXd& RobotDynTree::getFreeFloatingMassMatrix()
{
    assertInitialized();

    return impl_->getFreeFloatingMassMatrix();
}

const Eigen::MatrixXd& RobotDynTree::getMassMatrix()
{
    return impl_->getMassMatrix();
}

const Eigen::MatrixXd& RobotDynTree::getFrameFreeFloatingJacobian(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getFrameFreeFloatingJacobian(frameName);
}

const Eigen::MatrixXd& RobotDynTree::getJacobian(const std::string& frameName)
{
    return getRelativeJacobian(impl_->getBaseFrame(),frameName);
}

const Eigen::MatrixXd& RobotDynTree::getRelativeJacobian(const std::string& refFrameName, const std::string& frameName)
{
    assertFrameExists(refFrameName);
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getRelativeJacobian(refFrameName,frameName);
}

const Eigen::VectorXd& RobotDynTree::getJointPos() const
{
    assertInitialized();
    return impl_->getJointPos();
}

const Eigen::VectorXd& RobotDynTree::getJointVel() const
{
    assertInitialized();
    return impl_->getJointVel();
}

bool RobotDynTree::addAdditionalFrameToLink(const std::string& linkName, const std::string& frameName, const Eigen::Matrix4d& link_H_frame)
{
    assertFrameExists(frameName);
    return impl_->addAdditionalFrameToLink(linkName, frameName, link_H_frame);
}

const Eigen::VectorXd& RobotDynTree::getJointGravityTorques()
{
    assertInitialized();
    return impl_->getJointGravityTorques();
}

const Eigen::VectorXd& RobotDynTree::getJointCoriolisTorques()
{
    generalizedBiasForces();
    getJointGravityTorques();
    return impl_->getJointCoriolisTorques();
}

const Eigen::VectorXd& RobotDynTree::getJointGravityAndCoriolisTorques()
{
    generalizedBiasForces();
    return impl_->getJointGravityAndCoriolisTorques();
}

const Eigen::VectorXd& RobotDynTree::generalizedBiasForces()
{
    assertInitialized();
    return impl_->generalizedBiasForces();
}
