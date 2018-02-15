#include <orca/robot/RobotDynTree.h>
#include <orca/optim/OptimisationVector.h>
#include <exception>
#include <stdexcept>

using namespace orca::robot;
using namespace orca::optim;

RobotDynTree::RobotDynTree(const std::string& modelFile)
{
    global_gravity_vector_.setZero();
    if(!modelFile.empty())
        loadModelFromFile(modelFile);
}



bool RobotDynTree::loadModelFromFile(const std::string& modelFile)
{
    bool ok = kinDynComp_.loadRobotModelFromFile(modelFile);
    if( !ok || getNrOfDegreesOfFreedom() == 0 )
    {
        throw std::runtime_error("Could not load robot model");
        return false;
    }

    urdf_url_ = modelFile;

    const iDynTree::Model & model = kinDynComp_.model();

    robotData_.resize(model);
    eigRobotState_.resize(model.getNrOfDOFs());
    idynRobotState_.resize(model.getNrOfDOFs());
    eigRobotState_.setFixedBase();

    for(unsigned int i = 0 ; i < kinDynComp_.getNrOfDegreesOfFreedom() ; i++)
    {
        double min = 0,max = 0;
        if(kinDynComp_.model().getJoint(i)->hasPosLimits())
        {
            min = kinDynComp_.model().getJoint(i)->getMinPosLimit(0);
            max = kinDynComp_.model().getJoint(i)->getMaxPosLimit(0);
            joint_pos_limits_[i] = {min,max};
        }
    }

    OptimisationVector().buildControlVariablesMapping(model.getNrOfDOFs());

    return ok;
}

void RobotDynTree::print() const
{
    std::cout << "Robot Model " << std::endl;
    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfJoints() ; i++)
    {
        std::cout << "      Joint " << i << " " << kinDynComp_.getRobotModel().getJointName(i) << std::endl;
    }

    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfFrames() ; i++)
    {
        std::cout << "      Frame " << i << " " << kinDynComp_.getRobotModel().getFrameName(i) << std::endl;
    }
    for(unsigned int i=0; i < kinDynComp_.getRobotModel().getNrOfLinks() ; i++)
    {
        std::cout << "      Link " << i << " " << kinDynComp_.getRobotModel().getLinkName(i) << std::endl;
    }
}

bool RobotDynTree::isInitialized() const
{
    return is_initialized_ && getNrOfDegreesOfFreedom() > 0;
}

void RobotDynTree::setGravity(const Eigen::Vector3d& g)
{
    global_gravity_vector_ = g;
}

void RobotDynTree::setBaseFrame(const std::string& base_frame)
{
    if(!frameExists(base_frame))
        throw std::runtime_error("Frame is not part of the robot");
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
    
    if(getNrOfDegreesOfFreedom() == 0)
        throw std::runtime_error("Robot model is not loaded");

    if( base_frame_.empty())
        throw std::runtime_error("Base/FreeFloating frame is empty. Please use setBaseFrame before setting the robot state");

    if(global_gravity_vector_.isZero(0))
        throw std::runtime_error("Gravity vector is zero. Please use setGravity before setting the robot state");

    if(jointPos.size() != getNrOfDegreesOfFreedom())
        throw std::runtime_error(util::Formatter() << "JointPos size do not match with current configuration : provided " << jointPos.size() << ", expected " << getNrOfDegreesOfFreedom());

    if(jointVel.size() != getNrOfDegreesOfFreedom())
        throw std::runtime_error(util::Formatter() << "JointVel size do not match with current configuration : provided " << jointVel.size() << ", expected " << getNrOfDegreesOfFreedom());

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
    return base_frame_;
}

const std::string& RobotDynTree::getFileURL() const
{
    return urdf_url_;
}

unsigned int RobotDynTree::getNrOfDegreesOfFreedom() const
{
    return kinDynComp_.getNrOfDegreesOfFreedom();
}

unsigned int RobotDynTree::configurationSpaceDimension() const
{
    return 6 + kinDynComp_.getNrOfDegreesOfFreedom();
}

const iDynTree::Model& RobotDynTree::getRobotModel()
{
    return kinDynComp_.getRobotModel();
}

const std::map<unsigned int, std::pair<double,double> >& RobotDynTree::getJointPositionLimits()
{
    return joint_pos_limits_;
}

bool RobotDynTree::frameExists(const std::string& frame_name)
{
    if(kinDynComp_.getFrameIndex(frame_name) < 0)
    {
        return false;
    }
    return true;
}

std::string RobotDynTree::getJointName(unsigned int idx)
{
    return kinDynComp_.model().getJointName(idx);
}

unsigned int RobotDynTree::getNrOfJoints()
{
    return kinDynComp_.model().getNrOfJoints();
}

Eigen::Matrix<double,4,4,Eigen::RowMajor> RobotDynTree::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getRelativeTransform(refFrameName,frameName).asHomogeneousTransform());
}

const Eigen::Matrix<double,6,1> RobotDynTree::getFrameVel(const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getFrameVel(frameName));
}

Eigen::Map< const Eigen::Matrix<double,6,1> > RobotDynTree::getFrameBiasAcc(const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getFrameBiasAcc(frameName));
}

Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > RobotDynTree::getFreeFloatingMassMatrix()
{
    kinDynComp_.getFreeFloatingMassMatrix(robotData_.idynMassMatrix);
    return iDynTree::toEigen(robotData_.idynMassMatrix);
}

Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > RobotDynTree::getFrameFreeFloatingJacobian(const std::string& frameName)
{
    kinDynComp_.getFrameFreeFloatingJacobian(frameName,robotData_.idynJacobianFb);
    return iDynTree::toEigen(robotData_.idynJacobianFb);
}

Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > RobotDynTree::getRelativeJacobian(const std::string& refFrameName, const std::string& frameName)
{
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

const Eigen::VectorXd& RobotDynTree::generalizedBiasForces()
{
    kinDynComp_.generalizedBiasForces(robotData_.generalizedBiasForces);
    robotData_.eigGeneralizedBiasForces.head(6) = iDynTree::toEigen(robotData_.generalizedBiasForces.baseWrench());
    robotData_.eigGeneralizedBiasForces.tail(kinDynComp_.getNrOfDegreesOfFreedom()) = iDynTree::toEigen(robotData_.generalizedBiasForces.jointTorques());
    return robotData_.eigGeneralizedBiasForces;
}
