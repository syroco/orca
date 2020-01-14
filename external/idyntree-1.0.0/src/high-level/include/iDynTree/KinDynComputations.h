/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_KINDYNCOMPUTATIONS_H
#define IDYNTREE_KINDYNCOMPUTATIONS_H

#include <string>

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>


#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{

class VectorDynSize;
class MatrixDynSize;
class Transform;
class Twist;
class SpatialInertia;
class SpatialMomentum;
class ClassicalAcc;
class SpatialAcc;
class Wrench;
class Model;
class Traversal;
class Position;
class FreeFloatingGeneralizedTorques;

/**
 * \ingroup iDynTreeHighLevel
 *
 * \brief High level stateful class wrapping several kinematics and dynamics algorithms.
 *
 * The kinematics dynamics computations class is an high level class stateful to access
 * several algorithms related to kinematics and dynamics of free floating robot systems.
 *
 * This class supports three possible convention to express the floating base information :
 * the inertial, the body-fixed and the mixed convention.
 * To get more info on this three conventions, check section II.C  of
 * On  the  Base  Frame  Choice  in  Free-Floating  Mechanical  Systems
 * and  its  Connection  to  Centroidal  Dynamics
 * https://traversaro.github.io/preprints/changebase.pdf
 *
 */
class KinDynComputations {
private:
    struct KinDynComputationsPrivateAttributes;
    KinDynComputationsPrivateAttributes * pimpl;

    // copy is disabled for the moment
    KinDynComputations(const KinDynComputations & other);
    KinDynComputations& operator=(const KinDynComputations& other);

    // Make sure that (if necessary) forward kinematics is updated
    // If it was already called before the last call to setRobotState,
    // exits without further computations
    void computeFwdKinematics();

    // Make sure that (if necessary) mass matrix is updated
    // If it was already called before the last call to setRobotState,
    // exits without further computations
    void computeRawMassMatrixAndTotalMomentum();

    // Make sure that (if necessary) the kinematics bias acc is update
    // If it was already called before the last call to setRobotState,
    // exits without further computations
    void computeBiasAccFwdKinematics();

    // Invalidate the cache of intermediated results (called by setRobotState)
    void invalidateCache();

    // Resize internal data structures after a model has been successfully loaded
    void resizeInternalDataStructures();

public:

    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     *
     */
    KinDynComputations();


    /**
     * Destructor
     *
     */
    virtual ~KinDynComputations();
    //@}

    /**
     * @name Model loading and definition methods
     * This methods are used to load the structure of your model.
     *
     */
    //@{

    /**
     * Load the model of the robot from a iDynTree::Model class.
     *
     * @param model the model to use in this class.
     * @return true if all went ok, false otherwise.
     */
    bool loadRobotModel(const iDynTree::Model & model );

    /**
     * Load the model of the robot from an external file.
     *
     * @param filename path to the file to load
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    IDYNTREE_DEPRECATED_WITH_MSG("Use iDynTree::ModelLoader::loadRobotModelFromFile and pass the Model to loadRobotModel")
    bool loadRobotModelFromFile(const std::string & filename, const std::string & filetype="urdf");

    /**
     * Load the model of the robot  from a string.
     *
     * @param modelString string containg the model of the robot.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    IDYNTREE_DEPRECATED_WITH_MSG("Use iDynTree::ModelLoader::loadRobotModelFromString and pass the Model to loadRobotModel")
    bool loadRobotModelFromString(const std::string & modelString, const std::string & filetype="urdf");

    /**
     * Return true if the models for the robot have been correctly.
     *
     * @return True if the class has been correctly configure, false otherwise.
     */
    bool isValid() const;

    /**
     * Set the used FrameVelocityRepresentation.
     *
     * @see FrameVelocityRepresentation
     */
    bool setFrameVelocityRepresentation(const FrameVelocityRepresentation frameVelRepr) const;

    /**
     * @brief Get the used FrameVelocityRepresentation.
     * @see setFrameVelocityRepresentation
     */
    FrameVelocityRepresentation getFrameVelocityRepresentation() const;
    //@}


    /**
     * Get the number of internal degrees of freedom of the robot model used
     * in the class.
     * This return the *internal* degrees of freedom, because it does not include
     * the eventual 6 degrees of freedom usually associated with the floating base.
     *
     * @return the number of internal degrees of freedom of the model.
     */
    unsigned int getNrOfDegreesOfFreedom() const;

    /**
     * Get a human readable description of a given internal degree of freedom of the robot.
     *
     */
    std::string getDescriptionOfDegreeOfFreedom(int dof_index);

    /**
     * Get a human readable description of all the internal degrees of freedom of the robot.
     *
     * @return a std::string containing the description of the internal degrees of freedom.
     */
    std::string getDescriptionOfDegreesOfFreedom();


    /**
     * Get the number of links contained in the model.
     *
     */
    unsigned int getNrOfLinks() const;

    /**
     * Get the number of frames contained in the model.
     *
     * \note The number of frames is always greater than or equal to
     *       the number of links because every link has one link reference frame
     *       associated. An arbitrary number of additional reference frames
     *       can associated to a given link.
     */
    unsigned int getNrOfFrames() const;

    /**
     * Get the name of the link considered as the floating base.
     *
     * @return the name of the base link.
     */
    std::string getFloatingBase() const;

    /**
     * Set the link that is used as the floating base link.
     *
     *
     * @return true if all went well, false otherwise (for example if the link name was not found).
     */
    bool setFloatingBase(const std::string & floatingBaseName);


    //@}


    /**
     * @name Methods to access the underlyng model for the robot
     */
    //@{

    const Model & model() const;
    const Model & getRobotModel() const;

    //@}

    /**
     * @name Methods to obtain the sparity patterns of Jacobians and Hessians
     * @note Sparsity patterns does not depend on the particular joint configuration.
     * The only requirement is that the model has been loaded before
     */
    //@{

    /**
     * Returns the sparsity pattern of the relative Jacobian for the specified frames
     *
     * The resulting matrix has the same size of the free floating Jacobian (6 x #DoFs)
     * It is filled with only 0 and 1 with the following meaning:
     * - 0: the element will always have 0 (for every robot configuration)
     * - 1: it exists a robot configuration such that the element have a value different from zero.
     * @param refFrameIndex refence frame for the Jacobian
     * @param frameIndex Jacobian frame
     * @param outJacobianPattern the Jacobian sparsity pattern
     * @return true on success. False otherwise
     */
    bool getRelativeJacobianSparsityPattern(const iDynTree::FrameIndex refFrameIndex,
                                            const iDynTree::FrameIndex frameIndex,
                                            iDynTree::MatrixDynSize & outJacobianPattern) const;


    /**
     * Returns the sparsity pattern of the free floating Jacobian for the specified frame
     *
     * The resulting matrix has the same size of the free floating Jacobian (6 x 6 + #DoFs)
     * It is filled with only 0 and 1 with the following meaning:
     * - 0: the element will always have 0 (for every robot configuration)
     * - 1: it exists a robot configuration such that the element have a value different from zero.
     * @param frameIndex Jacobian frame
     * @param outJacobianPattern the Jacobian sparsity pattern
     * @return true on success. False otherwise
     */
    bool getFrameFreeFloatingJacobianSparsityPattern(const FrameIndex frameIndex,
                                                     iDynTree::MatrixDynSize & outJacobianPattern) const;


    //@}


    /**
      * @name Methods to submit the input data for dynamics computations.
      */
    //@{

    /**
     * @brief Set the (internal) joint positions.
     *
     * \note This method sets only the joint positions, leaving all the other components of the state to their previous value.
     *
     * @param[in] s A vector of dimension this->model().getNrOfPosCoords() .
     * @return true if all went well, false otherwise.
     */
    bool setJointPos(const iDynTree::VectorDynSize& s);

    /**
     * Set the state for the robot (floating base)
     *
     * @param world_T_base  the homogeneous transformation that transforms position vectors expressed in the base reference frame
     *                      in position frames expressed in the world reference frame (i.e. pos_world = world_T_base*pos_base .
     * @param s a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed with the convention specified by the used FrameVelocityConvention.
     * @param s_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param world_gravity a 3d vector of the gravity acceleration vector, expressed in the world/inertial frame.
     *
     */
    bool setRobotState(const iDynTree::Transform &world_T_base,
                       const iDynTree::VectorDynSize& s,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::VectorDynSize& s_dot,
                       const iDynTree::Vector3& world_gravity);

    /**
     * Set the state for the robot (fixed base)
     * Same as setRobotState, but with:
     *  world_T_base      = iDynTree::Transform::Identity()
     *  base_velocity     = iDynTree::Twist::Zero();
     *
     */
    bool setRobotState(const iDynTree::VectorDynSize &s,
                       const iDynTree::VectorDynSize &s_dot,
                       const iDynTree::Vector3& world_gravity);

    void getRobotState(iDynTree::Transform &world_T_base,
                       iDynTree::VectorDynSize& s,
                       iDynTree::Twist& base_velocity,
                       iDynTree::VectorDynSize& s_dot,
                       iDynTree::Vector3& world_gravity);

    void getRobotState(iDynTree::VectorDynSize &s,
                       iDynTree::VectorDynSize &s_dot,
                       iDynTree::Vector3& world_gravity);

    /**
     * Access the robot state.
     */
    iDynTree::Transform getWorldBaseTransform();
    iDynTree::Twist     getBaseTwist();

    bool getJointPos(iDynTree::VectorDynSize &q);
    bool getJointVel(iDynTree::VectorDynSize &dq);

    /**
     * Get the n+6 velocity of the model.
     * Obtained by stacking the output of getBaseTwist and of getJointVel .
     */
    bool getModelVel(iDynTree::VectorDynSize &nu);

    //@}

    /**
      * @name Methods to get transform information between frames in the model, given the current state.
      */
    //@{

    /**
     * Get the index corresponding to a given frame name.
     * @return a integer greater than or equal to zero if the frame exist,
     *         a negative integer otherwise.
     */
    int getFrameIndex(const std::string & frameName) const;

    /**
     * Get the frame name corresponding to a given frame index.
     *
     */
    std::string getFrameName(const iDynTree::FrameIndex frameIndex) const;

    /**
     * Return the transform where the frame is the frame
     * specified by frameIndex, and the reference frame is the world one
     * (world_H_frame).
     *
     */
    iDynTree::Transform getWorldTransform(const iDynTree::FrameIndex frameIndex);

    /**
     * Version of getWorldTransform where the frame is specified by name.
     *
     * @note For real time code use the method that takes an integer.
     *
     */
    iDynTree::Transform getWorldTransform(std::string frameName);


    /**
     * Return the transform where the frame is the frame
     * specified by frameIndex, and the reference frame is the one specified
     * by refFrameIndex (refFrame_H_frame).
     *
     */
    iDynTree::Transform getRelativeTransform(const iDynTree::FrameIndex refFrameIndex,
                                             const iDynTree::FrameIndex frameIndex);

     /**
     * Return the transform between the frame with the origin of the frameOriginIndex
     * and the orientation of frameOrientationIndex and the one with the origin
     * of refFrameOriginIndex and the orientation of refFrameOrientationIndex .
     *
     * In symbols we return the (refFrameOrigin,refFrameOrientation)_H_(frameOrigin,frameORientation)
     *
     * This is a variant of the getRelativeTransform in which the orientation and origin part of both
     * side of the transform are explicited.
     *
     * \todo provide mode detailed documentation.
     *
     */
    iDynTree::Transform getRelativeTransformExplicit(const iDynTree::FrameIndex refFrameOriginIndex,
                                                     const iDynTree::FrameIndex refFrameOrientationIndex,
                                                     const iDynTree::FrameIndex    frameOriginIndex,
                                                     const iDynTree::FrameIndex    frameOrientationIndex);

    /**
     * Version of getRelativeTransform where the frames are specified by name.
     *
     * @note For real time code use the method that takes an integer.
     *
     */
    iDynTree::Transform getRelativeTransform(const std::string & refFrameName,
                                             const std::string & frameName);

    //@}

    /**
      * @name Methods to get frame velocity information given the current state.
      */
    //@{

    /**
     * Return the frame velocity, with the convention specified by getFrameVelocityRepresentation .
     */
    iDynTree::Twist getFrameVel(const std::string & frameName);

    /**
     * Return the frame velocity, with the convention specified by getFrameVelocityRepresentation .
     */
    iDynTree::Twist getFrameVel(const FrameIndex frameIdx);

    /**
     * Return the frame acceleration, with the convention specified by getFrameVelocityRepresentation .
     *
     * @warning As this method recomputes the accelerations of all links for each call, it may be computationally expensive.
     */
    Vector6 getFrameAcc(const std::string & frameName,
                        const Vector6& baseAcc,
                        const VectorDynSize& s_ddot);

    /**
     * Return the frame acceleration, with the convention specified by getFrameVelocityRepresentation .
     *
     * @warning As this method recomputes the accelerations of all links for each call, it may be computationally expensive.
     */
    Vector6 getFrameAcc(const FrameIndex frameIdx,
                        const Vector6& baseAcc,
                        const VectorDynSize& s_ddot);

    bool getFrameFreeFloatingJacobian(const std::string & frameName,
                                      iDynTree::MatrixDynSize & outJacobian);

    bool getFrameFreeFloatingJacobian(const FrameIndex frameIndex,
                                      iDynTree::MatrixDynSize & outJacobian);




    /**
     * Return the relative Jacobian between the two frames
     *
     * The Jacobian maps the internal robot shape with the relative
     * velocity of refFrame w.r.t. frame expressed depending on the velocity representation, i.e
     * \f[
     *  v_{refFrame, frame} = J_{refFrame, frame}(s) \dot{s}
     * \f]
     *
     */
    bool getRelativeJacobian(const iDynTree::FrameIndex refFrameIndex,
                             const iDynTree::FrameIndex frameIndex,
                             iDynTree::MatrixDynSize & outJacobian);

    /**
     * Return the relative Jacobian between the two frames
     *
     * The Jacobian maps the internal robot shape with the relative
     * velocity of refFrame w.r.t. frame expressed in the specified frame, i.e
     * \f[
     *  {}^{expressedOriginFrame, [expressedOrientationFrame]} \mathrm{v}_{refFrame, frame} = {}^{expressedOriginFrame, [expressedOrientationFrame]} J_{refFrame, frame}(s) \dot{s}
     * \f]
     *
     * @param refFrameIndex reference frame
     * @param frameIndex considered frame
     * @param expressedOriginFrameIndex frame whose origin is used to express the Jacobian
     * @param expressedOrientationFrameIndex frame whose orientation is used to express the Jacobian
     * @return true on success, false otherwise
     */
    bool getRelativeJacobianExplicit(const iDynTree::FrameIndex refFrameIndex,
                                     const iDynTree::FrameIndex frameIndex,
                                     const iDynTree::FrameIndex expressedOriginFrameIndex,
                                     const iDynTree::FrameIndex expressedOrientationFrameIndex,
                                     iDynTree::MatrixDynSize & outJacobian);


    /**
     * Get the bias acceleration (i.e. acceleration not due to robot acceleration) of the frame velocity.
     *
     * This term is usually called \f$\dot{J} \nu\f$ or \f$\dot{J} \dot{q}\f$.
     */
    Vector6 getFrameBiasAcc(const FrameIndex frameIdx);

    /**
     * Get the bias acceleration (i.e. acceleration not due to robot acceleration) of the frame velocity.
     *
     * This term is usually called \f$\dot{J} \nu\f$ or \f$\dot{J} \dot{q}\f$.
     */
    Vector6 getFrameBiasAcc(const std::string & frameName);


    // Todo getFrameRelativeVel and getFrameRelativeJacobian to match the getRelativeTransform behaviour

    //@}


    /**
      * @name Methods to get quantities related to centroidal dynamics.
      *
      * For a precise definition of the quantities computed by this methods, please check:
      * S. Traversaro, D. Pucci, F. Nori
      * On the Base Frame Choice in Free-Floating Mechanical Systems and its Connection to Centroidal Dynamics
      * https://traversaro.github.io/preprints/changebase.pdf
      */
    //@{

    /**
     * Return the center of mass position.
     *
     * @return the center of mass position, expressed in the world/inertial frame.
     */
    iDynTree::Position getCenterOfMassPosition();

    /**
     * Return the center of mass velocity, with respect to the world/inertial frame.
     *
     * \note This is the time derivative of the quantity returned by getCenterOfMassPosition .
     *
     */
    iDynTree::Vector3 getCenterOfMassVelocity();

    /**
     * Return the center of mass jacobian, i.e. the 3 \times (n+6) matrix such that:
     *  getCenterOfMassVelocity() == getCenterOfMassJacobian() * \nu .
     *
     */
    bool getCenterOfMassJacobian(MatrixDynSize & comJacobian);

    /**
     * Return the center of mass bias acceleration.
     */
     Vector3 getCenterOfMassBiasAcc();

    /**
     * Get the average velocity of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note the linear part of this twist correspond to the getCenterOfMassVelocity only if the FrameVelocityConvention is set to MIXED.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    iDynTree::Twist getAverageVelocity();

    /**
     * Get the jacobian of the average velocity of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note the linear part of this jacobian correspond to the getCenterOfMassVelocity only if the FrameVelocityConvention is set to MIXED.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    bool getAverageVelocityJacobian(MatrixDynSize & avgVelocityJacobian);

    /**
     * Get the centroidal average velocity of the robot.
     *
     * The quantity is the average velocity returned by getAverageVelocity, but computed in the center of mass
     * and with the orientation of the FrameVelocityRepresentation used.
     * It we indicate with G the center of mass, it is expressed in (G[A]) for the mixed and inertial representation,
     * and in (G[B]) for the base body-fixed representation.
     *
     * \note the linear part of this twist correspond to the getCenterOfMassVelocity only if the FrameVelocityConvention is set to MIXED or INERTIAL.
     *
     */
    iDynTree::Twist getCentroidalAverageVelocity();

    /**
     * Get the jacobian of the centroidal average velocity of the robot.
     *
     * See the getCentroidalAverageVelocity method for more info on this.
     */
    bool getCentroidalAverageVelocityJacobian(MatrixDynSize & centroidalAvgVelocityJacobian);

    /**
     * Get the linear and angular momentum of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    iDynTree::SpatialMomentum getLinearAngularMomentum();

    /**
     * Get the linear and angular momentum of the robot.
     * The quantity is expressed in (B[A]), (A) or (B) depending on the FrameVelocityConvention used.
     *
     * \note Implementation incomplete, please refrain to use until this warning has been removed.
     */
    bool getLinearAngularMomentumJacobian(MatrixDynSize & linAngMomentumJacobian);

    /**
     * Get the centroidal (total) momentum of the robot.
     * If G is the center of mass, this quantity is expressed in (G[A]), (G[A]) or (G[B]) depending on the FrameVelocityConvention used.
     *
     */
    iDynTree::SpatialMomentum getCentroidalTotalMomentum();

    //@}


    /**
      * @name Methods to get quantities related to unconstrained free floating equation of motions.
      *
      * This methods permits to compute several quantities related to free floating equation of methods.
      * Note that this equations needs to be coupled with a description of the interaction between the model
      * and the enviroment (such as a contant model, a bilateral constraint on some links or by considering
      * some external forces as inputs) to actually obtain a dynamical system description of the mechanical model evolution.
      *
      * The equations of motion of a free floating mechanical system under the effect of a uniform gravitational field are:
      * \f[
      * M(q) \dot{\nu} +
      * C(q, \nu) \nu +
      * G(q)
      * =
      * \begin{bmatrix}
      * 0_{6\times1} \newline
      * \tau
      * \end{bmatrix}
      * +
      * \sum_{L \in \mathcal{L}}
      * J_L^T \mathrm{f}_L^x
      * \f]
      *
      * where:
      *
      * * \f$n_{PC}\f$ is the value returned by Model::getNrOfPosCoords,
      * * \f$n_{DOF}\f$ is the value returned by Model::getNrOfDOFs,
      * * \f$n_{L}\f$ is the value returned by Model::getNrOfLinks,
      * * \f$q \in \mathbb{R}^3 \times \textrm{SO}(3) \times \mathbb{R}^{n_{PC}}\f$ is the robot position,
      * * \f$\nu \in \mathbb{R}^{6+n_{DOF}}\f$ is the robot velocity,
      * * \f$\dot{\nu} \in \mathbb{R}^{6+n_{DOF}}\f$ is the robot acceleration,
      * * \f$M(q) \in \mathbb{R}^{(6+n_{DOF}) \times (6+n_{DOF})}\f$ is the free floating mass matrix,
      * * \f$C(q, \nu)  \in \mathbb{R}^{(6+n_{DOF}) \times (6+n_{DOF})}\f$ is the coriolis matrix,
      * * \f$G(q) \in \mathbb{R}^{6+n_{DOF}}\f$ is the vector of gravity generalized forces,
      * * \f$\tau \in \mathbb{R}^6\f$ is the vector of torques applied on the joint of the multibody model,
      * * \f$\mathcal{L}\f$ is the set of all the links contained in the multibody model,
      * * \f$J_L \in \mathbb{R}^{6+n_{DOF}}\f$ is the free floating jacobian of link \f$L\f$ as obtained by KinDynComputations::getFrameFreeFloatingJacobian,
      * * \f$\mathrm{f}_L^x\f$ is the 6D force/torque applied by the enviroment on link \f$L\f$.
      *
      * The precise definition of each quantity (in particular the part related to the base) actually depends on the
      * choice of FrameVelocityRepresentation, specified with the setFrameVelocityRepresentation method.
      *
      */
    //@{

    /**
     * @brief Get the free floating mass matrix of the system.
     *
     * This method computes \f$M(q) \in \mathbb{R}^{(6+n_{DOF}) \times (6+n_{DOF})}\f$.
     *
     * The mass matrix depends on the joint positions, specified by the setRobotState methods.
     * If the chosen FrameVelocityRepresentation is MIXED_REPRESENTATION or INERTIAL_FIXED_REPRESENTATION,
     * the mass matrix depends also on the base orientation with respect to the inertial frame,
     * that is also set by the  setRobotState methods.
     *
     * For more details on the structure of the free floating mass matrix, please check:
     * S. Traversaro, A. Saccon
     * Multibody Dynamics Notation
     * http://repository.tue.nl/849895
     *
     * @param[out] freeFloatingMassMatrix the (6+getNrOfDOFs()) times (6+getNrOfDOFs()) output mass matrix.
     * @return true if all went well, false otherwise.
     */
    bool getFreeFloatingMassMatrix(MatrixDynSize & freeFloatingMassMatrix);


    /**
     * @brief Compute the free floating inverse dynamics.
     *
     * This method computes \f$M(q) \dot{\nu} + C(q, \nu) \nu + G(q) - \sum_{L \in \mathcal{L}} J_L^T \mathrm{f}_L^x \in \mathbb{R}^{6+n_{DOF}}\f$.
     *
     * The semantics of baseAcc, the base part of baseForceAndJointTorques
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[in] baseAcc the acceleration of the base link
     * @param[in] s_ddot the accelerations of the joints
     * @param[in] linkExtForces the external wrenches excerted by the environment on the model
     * @param[out] baseForceAndJointTorques the output generalized torques
     * @return true if all went well, false otherwise
     */
    bool inverseDynamics(const Vector6& baseAcc,
                         const VectorDynSize& s_ddot,
                         const LinkNetExternalWrenches & linkExtForces,
                               FreeFloatingGeneralizedTorques & baseForceAndJointTorques);

    /**
     * @brief Compute the getNrOfDOFS()+6 vector of generalized bias (gravity+coriolis) forces.
     *
     * This method computes \f$C(q, \nu) \nu + G(q) \in \mathbb{R}^{6+n_{DOF}}\f$.
     *
     * The semantics of the base part of generalizedBiasForces depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[out] generalizedBiasForces the output generalized bias forces
     * @return true if all went well, false otherwise
     */
    bool generalizedBiasForces(FreeFloatingGeneralizedTorques & generalizedBiasForces);

    /**
     * @brief Compute the getNrOfDOFS()+6 vector of generalized gravity forces.
     *
     * This method computes \f$G(q) \in \mathbb{R}^{6+n_{DOF}}\f$.
     *
     * The semantics of the base part of generalizedGravityForces depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[out] generalizedGravityForces the output gravity generalized forces
     * @return true if all went well, false otherwise
     */
    bool generalizedGravityForces(FreeFloatingGeneralizedTorques & generalizedGravityForces);

    /**
     * @brief Compute the getNrOfDOFS()+6 vector of generalized external forces.
     *
     * This method computes \f$ -\sum_{L \in \mathcal{L}} J_L^T \mathrm{f}_L^x \in \mathbb{R}^{6+n_{DOF}} \f$.
     *
     * @warning Note that this method returns the **negated** sum of the product of jacobian and the external force,
     *          consistently with how the generalized external forces are computed in the KinDynComputations::inverseDynamics method.
     *
     * The semantics of the base part of generalizedExternalForces
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[out] generalizedExternalForces the output external generalized forces
     * @return true if all went well, false otherwise
     */
    bool generalizedExternalForces(const LinkNetExternalWrenches & linkExtForces,
                                         FreeFloatingGeneralizedTorques & generalizedExternalForces);

    /**
     * @brief Compute the free floating inverse dynamics as a linear function of inertial parameters.
     *
     * This methods computes the \f$ Y(\dot{\nu}, \nu, q) \in \mathbb{R}^{ (6+n_{DOF}) \times (10n_{L}) } \f$ matrix such that:
     * \f[
     *  Y(\dot{\nu}, \nu, q) \phi = M(q) \dot{\nu} + C(q, \nu) \nu + G(q)
     * \f]
     *
     * where \f$\phi \in \mathbb{R}^{10n_{L}}\f$ is the vector of inertial parameters returned by the Model::getInertialParameters .
     *
     * The semantics of baseAcc, the base part (first six rows) of baseForceAndJointTorquesRegressor
     * depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @see iDynTree::InverseDynamicsInertialParametersRegressor for more info on the underlying algorithm.
     *
     * @param[in] baseAcc the acceleration of the base link
     * @param[in] s_ddot the accelerations of the joints
     * @param[out] baseForceAndJointTorquesRegressor The (6+model.getNrOfDOFs() X 10*model.getNrOfLinks()) inverse dynamics regressor.
     * @return true if all went well, false otherwise
     */
    bool inverseDynamicsInertialParametersRegressor(const Vector6& baseAcc,
                                                    const VectorDynSize& s_ddot,
                                                          MatrixDynSize& baseForceAndJointTorquesRegressor);

    //@}


};

}

#endif

