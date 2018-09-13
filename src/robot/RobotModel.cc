#include "orca/robot/RobotModel.h"
#include "orca/utils/Utils.h"
#include "orca/math/Utils.h"
#include "RobotModel_iDynTree.impl"
#include <exception>
#include <stdexcept>
#include <string>
#include <fstream>
#include <streambuf>
#include <tinyxml.h>
#include <orca/common/Factory.h>

using namespace orca::robot;
using namespace orca::utils;
using namespace orca::math;
using namespace orca::common;

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
        orca_throw(Formatter() << "Frame '" << frame << "' is not part of the robot"); \
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

RobotModel::RobotModel(const std::string& robot_name)
: ConfigurableOrcaObject(robot_name)
, name_(robot_name)
{
    this->config()->onSuccess([&](){ loadFromParameters(); });
    this->addParameter("name",&name_,ParamPolicy::Optional);
    this->addParameter("base_frame",&base_frame_,ParamPolicy::Required);
    this->addParameter("urdf_url",&urdf_url_,ParamPolicy::Optional);
    this->addParameter("urdf_str",&urdf_str_,ParamPolicy::Optional);
    this->addParameter("gravity",&gravity_,ParamPolicy::Optional);
    this->addParameter("home_joint_positions",&home_joint_positions_,ParamPolicy::Optional);
    
    gravity_ = Eigen::Vector3d(0,0,-9.809);

    switch(robot_kinematics_type_)
    {
        case RobotModelImplType::iDynTree:
            impl_ = make_unique<RobotModelImpl<iDynTree> >();
            break;
        default:
            orca_throw(Formatter() << "RobotModel only support iDynTree for now");
    }
}

bool RobotModel::loadFromParameters()
{
    if(!urdf_url_.isSet() && !urdf_str_.isSet())
        orca_throw(Formatter() << "urdf_str and urdf_url are not set !\nYou should at least provide one of them to load the robot model.");
    
    if(urdf_url_.isSet())
    {
        if(!this->loadModelFromFile(urdf_url_.get()))
            return false;
    }
    else if(urdf_str_.isSet())
    {
        if(!this->loadModelFromString(urdf_str_.get()))
            return false;
    }
    
    this->setBaseFrame(base_frame_.get());

    if(gravity_.isSet())
        this->setGravity(gravity_.get());

    if(home_joint_positions_.isSet())
    {
        if(home_joint_positions_.get().size() != getNrOfDegreesOfFreedom())
            orca_throw(Formatter() << "home_joint_positions provided does not match ndof (" 
                << home_joint_positions_.get().size() 
                << " vs " << getNrOfDegreesOfFreedom() << ")"
            );
        
        Eigen::VectorXd zero_vel = Eigen::VectorXd::Zero(getNrOfDegreesOfFreedom());

        // Set the first state to the robot
        this->setRobotState(home_joint_positions_.get(),zero_vel);
    }
    return true;
}

RobotModel::~RobotModel()
{

}

const std::vector<std::string>& RobotModel::getLinkNames() const
{
    return impl_->getLinkNames();
}

const std::vector<std::string>& RobotModel::getFrameNames() const
{
    return impl_->getFrameNames();
}

const std::vector<std::string>& RobotModel::getJointNames() const
{
    return impl_->getJointNames();
}

bool RobotModel::loadModelFromString(const std::string &modelString)
{
    if( modelString.empty() )
    {
        LOG_ERROR << "Model string is empty";
        return false;
    }
    // Extract the model name from the URDF
    // WARNING : in multi robot environnement + ROS
    // This will cause topic names collisions as they are based on robot names
    if(name_.get().empty())
    {
        // If no name is provided, let's find it on the URDF
        TiXmlDocument doc;
        doc.Parse(modelString.c_str());
        
        std::string name_in_xml;
        
        if(!getRobotNameFromTinyXML(&doc,name_in_xml))
        {
            std::cerr << "modelString : \n" << modelString << '\n';
            LOG_ERROR << "Could not extract automatically the robot name from the urdf." \
                << '\n'
                << "Please use auto robot = std::make_shared<RobotModel>(\"my_robot_name\")";
        }
        else
        {
            LOG_DEBUG << "Name extracted from URDF string : " << name_in_xml;
        }
        
        // Set the new name
        name_.set( name_in_xml );
    }
    if(impl_->loadModelFromString(modelString))
    {
        urdf_str_ = modelString;
        LOG_INFO << "Robot model '" << getName() << "' (" << getNrOfDegreesOfFreedom() << " dof) successfully loaded";
        return true;
    }
    std::cerr << "modelString : \n" << modelString << "\n\n";
    orca_throw(Formatter() << modelString << "\n\nCould not load robot model from above urdf string.\n\n");
    return false;
}

const std::string& RobotModel::getName() const
{
    return name_.get();
}

bool RobotModel::loadModelFromFile(const std::string &modelFile)
{
    std::ifstream t(modelFile);
    if( !t.good() )
    {
        LOG_ERROR << "Could not load model from urdf file \'" << modelFile << "\' because file is empty";
        return false;
    }
    std::string str((std::istreambuf_iterator<char>(t)),
                     std::istreambuf_iterator<char>());

    if(loadModelFromString(str))
    {
        urdf_url_ = modelFile;
        return true;
    }
    return false;
}

const std::string& RobotModel::getUrdfUrl() const
{
    if(urdf_url_.get().empty() && !urdf_str_.get().empty())
        LOG_WARNING << "Robot model has been loaded with a URDF string, so the url is empty";
    assertLoaded();
    return urdf_url_.get();
}

const std::string& RobotModel::getUrdfString() const
{
    if(urdf_str_.get().empty())
        LOG_ERROR << "Robot is not loaded, URDF string is empty";
    return urdf_str_.get();
}

const Eigen::VectorXd& RobotModel::getMinJointPos()
{
    return impl_->getMinJointPos();
}
const Eigen::VectorXd& RobotModel::getMaxJointPos()
{
    return impl_->getMaxJointPos();
}

void RobotModel::print() const
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

void RobotModel::onRobotInitializedCallback(std::function<void(void)> cb)
{
    robot_initialized_cb_ = cb;
}

bool RobotModel::isInitialized() const
{
    return is_initialized_ && getNrOfDegreesOfFreedom() > 0;
}

void RobotModel::setGravity(const Eigen::Vector3d& g)
{
    impl_->setGravity(g);
}

void RobotModel::setBaseFrame(const std::string& base_frame)
{
    assertFrameExists(base_frame);
    return impl_->setBaseFrame( base_frame );
}

void RobotModel::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,impl_->getGravity());
}
void RobotModel::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    setRobotState(impl_->getWorldToBaseTransform(),jointPos,impl_->getBaseVelocity(),jointVel,gravity);
}

void RobotModel::setRobotState(const Eigen::Matrix4d& world_H_base
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
        LOG_DEBUG << "Robot \'" << getName() << "\' is now initialized";
        is_initialized_ = true;
        if(robot_initialized_cb_)
            robot_initialized_cb_();
    }
}

const std::string& RobotModel::getBaseFrame() const
{
    assertLoaded();

    if(impl_->getBaseFrame().empty())
        orca_throw("BaseFrame is empty. Please robot->setBaseFrame(\"some_frame_on_the_robot_that_attach_him_to_the_ground\")");

    return impl_->getBaseFrame();
}

unsigned int RobotModel::getNrOfDegreesOfFreedom() const
{
    assertLoaded();
    return impl_->getNrOfDegreesOfFreedom();
}

unsigned int RobotModel::getConfigurationSpaceDimension() const
{
    return 6 + getNrOfDegreesOfFreedom();
}

bool RobotModel::frameExists(const std::string& frame_name) const
{
    assertLoaded();
    return impl_->frameExists(frame_name);
}

std::string RobotModel::getJointName(unsigned int idx) const
{
    assertLoaded();
    return impl_->getJointName(idx);
}

unsigned int RobotModel::getNrOfJoints() const
{
    assertLoaded();
    return impl_->getNrOfJoints();
}

const Eigen::Matrix4d& RobotModel::getTransform(const std::string& frameName)
{
    return getRelativeTransform(impl_->getBaseFrame(),frameName);
}

const Eigen::Matrix4d& RobotModel::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    assertFrameExists(refFrameName);
    assertFrameExists(frameName);
    assertInitialized();
    return impl_->getRelativeTransform(refFrameName,frameName);
}

const Eigen::Matrix<double,6,1>&  RobotModel::getFrameVel(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getFrameVel(frameName);
}

const Eigen::Matrix<double,6,1>& RobotModel::getFrameBiasAcc(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getFrameBiasAcc(frameName);
}

const Eigen::MatrixXd& RobotModel::getFreeFloatingMassMatrix()
{
    assertInitialized();

    return impl_->getFreeFloatingMassMatrix();
}

const Eigen::MatrixXd& RobotModel::getMassMatrix()
{
    return impl_->getMassMatrix();
}

const Eigen::MatrixXd& RobotModel::getFrameFreeFloatingJacobian(const std::string& frameName)
{
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getFrameFreeFloatingJacobian(frameName);
}

const Eigen::MatrixXd& RobotModel::getJacobian(const std::string& frameName)
{
    return getRelativeJacobian(impl_->getBaseFrame(),frameName);
}

const Eigen::MatrixXd& RobotModel::getRelativeJacobian(const std::string& refFrameName, const std::string& frameName)
{
    assertFrameExists(refFrameName);
    assertFrameExists(frameName);
    assertInitialized();

    return impl_->getRelativeJacobian(refFrameName,frameName);
}

const Eigen::VectorXd& RobotModel::getJointPos() const
{
    assertInitialized();
    return impl_->getJointPos();
}

const Eigen::VectorXd& RobotModel::getJointVel() const
{
    assertInitialized();
    return impl_->getJointVel();
}

bool RobotModel::addAdditionalFrameToLink(const std::string& linkName, const std::string& frameName, const Eigen::Matrix4d& link_H_frame)
{
    assertFrameExists(frameName);
    return impl_->addAdditionalFrameToLink(linkName, frameName, link_H_frame);
}

const Eigen::VectorXd& RobotModel::getJointGravityTorques()
{
    assertInitialized();
    return impl_->getJointGravityTorques();
}

const Eigen::VectorXd& RobotModel::getJointCoriolisTorques()
{
    generalizedBiasForces();
    getJointGravityTorques();
    return impl_->getJointCoriolisTorques();
}

const Eigen::VectorXd& RobotModel::getJointGravityAndCoriolisTorques()
{
    generalizedBiasForces();
    return impl_->getJointGravityAndCoriolisTorques();
}

const Eigen::VectorXd& RobotModel::generalizedBiasForces()
{
    assertInitialized();
    return impl_->generalizedBiasForces();
}

ORCA_REGISTER_CLASS(orca::robot::RobotModel)
