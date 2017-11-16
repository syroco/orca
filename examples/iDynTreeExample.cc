/**
* @ingroup idyntree_tutorials
*
* A tutorial on how to use the KinDynComputations class
* with Eigen data structures.
*
* \author Silvio Traversaro
*
* CopyPolicy: Released under the terms of LGPL 2.0+ or later
*/
// #include <rtt/os/main.h>
//
// #include <rtt/TaskContext.hpp>
// #include <rtt/Logger.hpp>
// #include <rtt/Property.hpp>
// #include <rtt/Attribute.hpp>
// #include <rtt/OperationCaller.hpp>
// #include <rtt/Port.hpp>
// #include <rtt/Activity.hpp>
//
// #include <ocl/TaskBrowser.hpp>
// #include <ocl/DeploymentComponent.hpp>
// #include <ocl/OCL.hpp>

// using namespace RTT;
// C headers
#include <cstdlib>

// Eigen headers
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>


/**
 * Struct containing the floating robot state
 * using Eigen data structures.
 */
struct EigenRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        world_H_base.setIdentity();
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
        jointPos.setZero();
        jointVel.setZero();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    Eigen::Matrix4d world_H_base;
    Eigen::VectorXd jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    Eigen::VectorXd jointVel;
    Eigen::Vector3d gravity;
};

/**
 * Struct containing the floating robot state
 * using iDynTree data structures.
 * For the semantics of this structures,
 * see KinDynComputation::setRobotState method.
 */
struct iDynTreeRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
        jointPos.zero();
        jointVel.zero();
    }

    iDynTree::Transform world_H_base;
    iDynTree::VectorDynSize jointPos;
    iDynTree::Twist         baseVel;
    iDynTree::VectorDynSize jointVel;
    iDynTree::Vector3       gravity;
};

struct EigenRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
        baseAcc.setZero();
        jointAcc.setZero();
    }

    void random()
    {
        baseAcc.setRandom();
        jointAcc.setRandom();
    }

    Eigen::Matrix<double,6,1> baseAcc;
    Eigen::VectorXd jointAcc;
};

struct iDynTreeRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
        baseAcc.zero();
        jointAcc.zero();
    }

    iDynTree::Vector6 baseAcc;
    iDynTree::VectorDynSize jointAcc;
};


/*****************************************************************/
// int ORO_main(int argc, char** argv)
// {
//     OCL::DeploymentComponent console_deployer("Deployer");
//
int main(int argc, char** argv)
{

    std::string modelFile = "/home/hoarau/isir/orca_ws/src/orca/examples/lwr4.urdf";

    // Helper class to load the model from an external format
    iDynTree::ModelLoader mdlLoader;

    bool ok = mdlLoader.loadModelFromFile(modelFile);

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return EXIT_FAILURE;
    }

    // Create a KinDynComputations class from the model
    iDynTree::KinDynComputations kinDynComp;
    ok = kinDynComp.loadRobotModel(mdlLoader.model());
    kinDynComp.setFloatingBase("link_0");
    
    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
    }

    const iDynTree::Model & model = kinDynComp.model();

    // Now we create the robot state in Eigen datastructures
    EigenRobotState eigRobotState;
    eigRobotState.resize(model.getNrOfDOFs());
    //eigRobotState.random();

    // Now we convert the Eigen data structures in iDynTree ones
    iDynTreeRobotState idynRobotState;
    idynRobotState.resize(model.getNrOfDOFs());
    iDynTree::fromEigen(idynRobotState.world_H_base,eigRobotState.world_H_base);
    iDynTree::toEigen(idynRobotState.jointPos) = eigRobotState.jointPos;
    iDynTree::fromEigen(idynRobotState.baseVel,eigRobotState.baseVel);
    toEigen(idynRobotState.jointVel) = eigRobotState.jointVel;
    toEigen(idynRobotState.gravity)  = eigRobotState.gravity;

    // Now we create the KinDynComputations class, so we can set the state
    std::cout << "Setting state : \n" << idynRobotState.world_H_base.toString() << '\n';
    
    kinDynComp.setRobotState(idynRobotState.world_H_base,idynRobotState.jointPos,
                             idynRobotState.baseVel,idynRobotState.jointVel,idynRobotState.gravity);

    // Once we called the setRobotState, we can call all the methods of KinDynComputations

    // For methods returning fixed size vector/matrices, the conversion to eigen types is trivial
    Eigen::Vector3d com = iDynTree::toEigen(kinDynComp.getCenterOfMassPosition());

    // If you are interested in a frame with a given name, you can obtain its associated index with
    // the appropriate method
    std::string arbitraryFrameName = model.getFrameName(model.getNrOfFrames()/2);

    iDynTree::FrameIndex arbitraryFrameIndex = model.getFrameIndex(arbitraryFrameName);

    Eigen::Matrix4d world_H_arbitraryFrame = iDynTree::toEigen(kinDynComp.getWorldTransform(arbitraryFrameIndex).asHomogeneousTransform());

    // You can also get quantities directly with their name, but clearly this is less efficient
    std::string anotherArbitraryFrameName = model.getFrameName(model.getNrOfFrames()/3);

    Eigen::Matrix4d arbitraryFrame_H_anotherArbitraryFrame = iDynTree::toEigen(kinDynComp.getRelativeTransform(arbitraryFrameName,anotherArbitraryFrameName).asHomogeneousTransform());

    // More complex quantities (such as jacobians and matrices) need to be handled in a different way for efficency reasons
    iDynTree::FreeFloatingMassMatrix idynMassMatrix(model);
    kinDynComp.getFreeFloatingMassMatrix(idynMassMatrix);
    Eigen::MatrixXd eigMassMatrix = iDynTree::toEigen(idynMassMatrix);

    std::cerr << "FreeFloatingMassMatrix: \n" << std::endl;
    std::cerr << eigMassMatrix << std::endl;

    iDynTree::FrameFreeFloatingJacobian idynJacobian(model);
    kinDynComp.getFrameFreeFloatingJacobian("link_7",idynJacobian);
    Eigen::MatrixXd eigJacobian = iDynTree::toEigen(idynJacobian);

    std::cerr << "getFrameFreeFloatingJacobian: \n" << std::endl;
    std::cerr << eigJacobian << std::endl;

    iDynTree::MatrixDynSize idynCOMJacobian(3,model.getNrOfDOFs()+6);
    kinDynComp.getCenterOfMassJacobian(idynCOMJacobian);
    Eigen::MatrixXd eigCOMJacobian = iDynTree::toEigen(idynCOMJacobian);

    std::cerr << "COM Jacobian: " << std::endl;
    std::cerr << eigCOMJacobian << std::endl;

    // For inverse dynamics, we need to pass the acceleration, of the robot
    // as it is not part of the "state"
    EigenRobotAcceleration eigRobotAcc;
    eigRobotAcc.resize(model.getNrOfDOFs());
    //eigRobotAcc.random();

    iDynTreeRobotAcceleration idynRobotAcc;
    idynRobotAcc.resize(model.getNrOfDOFs());
    iDynTree::toEigen(idynRobotAcc.baseAcc) = eigRobotAcc.baseAcc;
    iDynTree::toEigen(idynRobotAcc.jointAcc) = eigRobotAcc.jointAcc;

    // In input to the inverse dynamics we also have external forces (that we assume set to zero)
    iDynTree::LinkNetExternalWrenches extForces(model);
    extForces.zero();

    // The output is a set of generalized torques (joint torques + base wrenches)
    iDynTree::FreeFloatingGeneralizedTorques invDynTrqs(model);

    kinDynComp.inverseDynamics(idynRobotAcc.baseAcc,idynRobotAcc.jointAcc,extForces,invDynTrqs);

    iDynTree::Vector6 jdot_qdot = kinDynComp.getFrameBiasAcc(model.getNrOfFrames() - 1);
    std::cout << "Jacobian point q point = " << std::endl << iDynTree::toEigen(jdot_qdot) << std::endl;

    // The output of inv dynamics can be converted easily to eigen vectors
    Eigen::Matrix<double,6,1> baseWrench = iDynTree::toEigen(invDynTrqs.baseWrench());
    Eigen::VectorXd jntTorques = iDynTree::toEigen(invDynTrqs.jointTorques());

    std::cerr << "Inverse dynamics torques: " << std::endl;
    std::cerr << jntTorques << std::endl;

//     log(Info) << "**** Starting the TaskBrowser       ****" <<endlog();
//     // Switch to user-interactive mode.
//     OCL::TaskBrowser browser( &console_deployer );
//
//     // Accept user commands from console.
//     browser.loop();


    return EXIT_SUCCESS;
}
