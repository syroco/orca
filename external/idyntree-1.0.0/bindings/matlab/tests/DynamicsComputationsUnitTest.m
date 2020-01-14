function test_suite=DynamicsComputationsUnitTest
    initTestSuite

function test_inverse_dynamics_consistency
    tol = 1e-9;

    dynComp = iDynTree.DynamicsComputations();

    ok = dynComp.loadRobotModelFromFile('./model.urdf');

    if not(ok)
        fprintf('Skipping test_inverse_dynamics_consistency because iDynTree is compiled with IDYNTREE_USES_KDL to OFF');
        return;
    end

    dynComp.getFloatingBase();

    % set state
    dofs = dynComp.getNrOfDegreesOfFreedom();
    q = iDynTree.VectorDynSize(dofs);
    dq = iDynTree.VectorDynSize(dofs);
    ddq = iDynTree.VectorDynSize(dofs);
    
    q.fromMatlab(rand(dofs,1));
    dq.fromMatlab(rand(dofs,1));
    ddq.fromMatlab(rand(dofs,1));
    
    % set gravity
    grav = iDynTree.SpatialAcc();
    grav.setVal(0,0.0);
    grav.setVal(1,0.0);
    grav.setVal(2,-9.81);
    grav.setVal(3,0.0);
    grav.setVal(4,0.0);
    grav.setVal(5,0.0);
    
    dynComp.setRobotState(q,dq,ddq,grav);
    
    torques = iDynTree.VectorDynSize(dofs);
    baseReactionForce = iDynTree.Wrench();
    
    % compute id with inverse dynamics
    dynComp.inverseDynamics(torques,baseReactionForce);
    
    % compute id with regressors
    nrOfParams = 6*dynComp.getNrOfLinks();
    regr = iDynTree.MatrixDynSize(6+dofs,nrOfParams);
    params = iDynTree.VectorDynSize(nrOfParams);
    dynComp.getDynamicsRegressor(regr);
    dynComp.getModelDynamicsParameters(params);
    
    generalizedForces = regr.toMatlab()*params.toMatlab();
    
    % check consistency
    assertElementsAlmostEqual([baseReactionForce.toMatlab();torques.toMatlab()],generalizedForces);
    