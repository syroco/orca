#include <orca/robot/RobotDynTree.h>
#include <orca/optim/OptimisationVector.h>
#include <exception>
#include <stdexcept>

using namespace orca::robot;
using namespace orca::optim;

RobotDynTree::RobotDynTree(const std::string& modelFile)
{
    global_gravity_vector.setZero();
    if(!modelFile.empty())
        loadModelFromFile(modelFile);
}



bool RobotDynTree::loadModelFromFile(const std::string& modelFile)
{
    urdf_url = modelFile;
    bool ok = kinDynComp.loadRobotModelFromFile(modelFile);
    if( !ok || getNrOfDegreesOfFreedom() == 0 )
    {
        throw std::runtime_error("Could not load robot model");
        return false;
    }

    const iDynTree::Model & model = kinDynComp.model();

    eigRobotState.resize(model.getNrOfDOFs());
    idynRobotState.resize(model.getNrOfDOFs());
    eigRobotState.setFixedBase();

    OptimisationVector().buildControlVariablesMapping(model.getNrOfDOFs());

    is_initialized = ok;
    return ok;
}

void RobotDynTree::print()
{
    std::cout << "Robot Model " << std::endl;
    for(int i=0; i < kinDynComp.model().getNrOfJoints() ; i++)
    {
        std::cout << "      Joint " << i << " " << kinDynComp.model().getJointName(i) << std::endl;
    }

    for(int i=0; i < kinDynComp.model().getNrOfFrames() ; i++)
    {
        std::cout << "      Frame " << i << " " << kinDynComp.model().getFrameName(i) << std::endl;
    }
    for(int i=0; i < kinDynComp.model().getNrOfLinks() ; i++)
    {
        std::cout << "      Link " << i << " " << kinDynComp.model().getLinkName(i) << std::endl;
    }
}

void RobotDynTree::setGravity(const Eigen::Vector3d& g)
{
    global_gravity_vector = g;
}

void RobotDynTree::setBaseFrame(const std::string& base_frame)
{
    if(!frameExists(base_frame))
        throw std::runtime_error("Frame is not part of the robot");
    baseFrame = base_frame;
    kinDynComp.setFloatingBase( baseFrame );
}

void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,global_gravity_vector);
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
    if( baseFrame.empty())
        throw std::runtime_error("Base/FreeFloating frame is empty. Please use setBaseFrame before setting the robot state");

    if(global_gravity_vector.mean() == 0)
        throw std::runtime_error("Global gravity vector is not set. Please use setGravity before setting the robot state");

    eigRobotState.world_H_base = world_H_base;
    eigRobotState.jointPos = jointPos;
    eigRobotState.baseVel = baseVel;
    eigRobotState.jointVel = jointVel;
    eigRobotState.gravity = gravity;

    iDynTree::fromEigen(idynRobotState.world_H_base,eigRobotState.world_H_base);
    iDynTree::toEigen(idynRobotState.jointPos) = eigRobotState.jointPos;
    iDynTree::fromEigen(idynRobotState.baseVel,eigRobotState.baseVel);
    iDynTree::toEigen(idynRobotState.jointVel) = eigRobotState.jointVel;
    iDynTree::toEigen(idynRobotState.gravity)  = eigRobotState.gravity;

    kinDynComp.setRobotState(idynRobotState.world_H_base
                            ,idynRobotState.jointPos
                            ,idynRobotState.baseVel
                            ,idynRobotState.jointVel
                            ,idynRobotState.gravity);
}
