function test_suite = highLevelWrappersSmokeTest
    initTestSuite

function test__high_level_wrappers
   
    % specify the list of joints that are going to be considered in the reduced model
    jointList    = {'torso_pitch','torso_roll','torso_yaw',...
                    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow','l_wrist_prosup', ...
                    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow','r_wrist_prosup', ...
                    'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                    'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
        
    % select the link that will be used as base link
    baseLinkName = 'root_link';

    % urdf model name (expected to be in the same folder of this test)
    modelName    = 'model.urdf';

    % set the initial robot position and velocity [deg]
    torso_Position     = [0  0  0];                 
    left_arm_Position  = [10 45 0  15 0];           
    right_arm_Position = [10 45 0  15 0];         
    right_leg_Position = [0  0  0  0  0  0];        
    left_leg_Position  = [0  0  0  0  0  0];

    jointPos_init = [torso_Position';left_arm_Position';right_arm_Position';left_leg_Position';right_leg_Position']*pi/180;
    jointVel_init = zeros(length(jointPos_init),1);

    % other configuration parameters
    gravityAcc    = [0;0;-9.81];
    frameVelRepr  = 'mixed';
    frameName     = 'l_sole';
    frame2Name    = 'r_sole';
    frameID       = 1;
    frame2ID      = 2;
          
    %% TESTS LIST
    try
        % test 1
        disp('1/28: testing loadReducedModel...')
        KinDynModel = iDynTreeWrappers.loadReducedModel(jointList, baseLinkName, './', modelName, true);

        % test 2
        disp('2/28: testing setRobotState...')
        iDynTreeWrappers.setRobotState(KinDynModel, jointPos_init, jointVel_init, gravityAcc)

        % test 3
        disp('3/28: testing setJointPos...')
        iDynTreeWrappers.setJointPos(KinDynModel, jointPos_init) 

        % test 4
        disp('4/28: testing setFrameVelocityRepresentation...')
        iDynTreeWrappers.setFrameVelocityRepresentation(KinDynModel, frameVelRepr)

        % test 5
        disp('5/28: testing setFloatingBase...')
        iDynTreeWrappers.setFloatingBase(KinDynModel, baseLinkName)
 
        % test 6
        disp('6/28: testing getJointPos...')
        iDynTreeWrappers.getJointPos(KinDynModel);

        % test 7
        disp('7/28: testing getJointVel...')
        iDynTreeWrappers.getJointVel(KinDynModel);

        % test 8
        disp('8/28: testing getCentroidalTotalMomentum...')
        iDynTreeWrappers.getCentroidalTotalMomentum(KinDynModel);
    
        % test 9
        disp('9/28: testing getNrOfDegreesOfFreedom...')
        iDynTreeWrappers.getNrOfDegreesOfFreedom(KinDynModel);

        % test 10
        disp('10/28: testing getCenterOfMassPosition...')
        iDynTreeWrappers.getCenterOfMassPosition(KinDynModel);

        % test 11
        disp('11/28: testing getBaseTwist...')
        iDynTreeWrappers.getBaseTwist(KinDynModel);

        % test 12
        disp('12/28: testing generalizedBiasForces...')
        iDynTreeWrappers.generalizedBiasForces(KinDynModel);

        % test 13
        disp('13/28: testing generalizedGravityForces...')
        iDynTreeWrappers.generalizedGravityForces(KinDynModel);

        % test 14
        disp('14/28: testing getWorldBaseTransform...')
        iDynTreeWrappers.getWorldBaseTransform(KinDynModel);

        % test 15
        disp('15/28: testing getModelVel...')
        iDynTreeWrappers.getModelVel(KinDynModel);
    
        % test 16
        disp('16/28: testing getFrameVelocityRepresentation...')
        iDynTreeWrappers.getFrameVelocityRepresentation(KinDynModel);

        % test 17
        disp('17/28: testing getFloatingBase...')
        iDynTreeWrappers.getFloatingBase(KinDynModel);

        % test 18
        disp('18/28: testing getFrameIndex...')
        iDynTreeWrappers.getFrameIndex(KinDynModel, frameName);

        % test 19
        disp('19/28: testing getFrameName...')
        iDynTreeWrappers.getFrameName(KinDynModel, frameID);

        % test 20
        disp('20/28: testing getWorldTransform...')
        iDynTreeWrappers.getWorldTransform(KinDynModel, frameName);

        % test 21
        disp('21/28: testing getRelativeTransform...')
        iDynTreeWrappers.getRelativeTransform(KinDynModel, frameName, frame2Name);

        % test 22
        disp('22/28: testing getRelativeJacobian...')
        iDynTreeWrappers.getRelativeJacobian(KinDynModel, frameID, frame2ID);

        % test 23
        disp('23/28: testing getFreeFloatingMassMatrix...')
        iDynTreeWrappers.getFreeFloatingMassMatrix(KinDynModel);

        % test 24
        disp('24/28: testing getRobotState...')
        iDynTreeWrappers.getRobotState(KinDynModel);

        % test 25
        disp('25/28: testing getFrameBiasAcc...')
        iDynTreeWrappers.getFrameBiasAcc(KinDynModel, frameName);

        % test 26
        disp('26/28: testing getCenterOfMassJacobian...')
        iDynTreeWrappers.getCenterOfMassJacobian(KinDynModel);

        % test 27
        disp('27/28: testing getCenterOfMassVelocity...')
        iDynTreeWrappers.getCenterOfMassVelocity(KinDynModel);
        
        % test 28
        disp('28/28: testing getFrameFreeFloatingJacobian...')
        iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel, frameName);
        
    catch ME
       disp('[High Level Wappers]: test failed. Message: ')
       disp(ME)
       assertTrue(false);
    end
