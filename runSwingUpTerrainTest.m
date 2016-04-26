function runSwingUpTerrainTest()
    % construct the plant with terrain
    options.view = 'right';
    terrainHeight = -2;
    options.terrain = RigidBodyFlatTerrain(terrainHeight);
    plant = PlanarRigidBodyManipulator('TestPlant.urdf',options);
    plant = plant.setInputLimits(-40,40);
    % construct the visualizer
    v = plant.constructVisualizer();
    v.axis = 5*[-1 1 -1 1];
    %v.inspector();
    %return;
    
    % set start and end points
    manipulator_state_0 = [-pi/2;0];
    manipulator_state_f = [pi;0];
    manipulator_vel_0 = zeros(2,1);
    manipulator_vel_f = zeros(2,1);
    x0 = [manipulator_state_0;manipulator_vel_0];
    xf = [manipulator_state_f;manipulator_vel_f];
    
    % timespan and number of colocation points
    tf0 = 4;
    N = 40;
    
    % create the contact implicit trajectory object
    prog = ContactImplicitTrajectoryOptimization(plant,N,[2 6]);
    % add the start and end state
    prog = prog.addStateConstraint(ConstantConstraint(x0),1);
    prog = prog.addStateConstraint(ConstantConstraint(xf),N);
    % add the running and final cost functions
    prog = prog.addRunningCost(@cost);
    prog = prog.addFinalCost(@finalCost);
    % add in the joint limit constraints
    joint_limits = BoundingBoxConstraint(plant.joint_limit_min,plant.joint_limit_max);
    prog = prog.addStateConstraint(joint_limits,[1:N],[1:size(plant.joint_limit_min,1)]);
    % initialize the trajectory
    traj_init.x = initTraj(x0,xf,tf0,N);
    
    % try 10 interations improving guess each time
    for attempts=1:10
        disp('Attempt #');
        disp(attempts);
        tic
        [xtraj,utraj,ltraj,ljltraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        % if done break
        if info==1, break; end
        % else improve the guess
        [traj_init.x, traj_init.u] = improveGuess(xtraj,utraj);
        v.playback(xtraj);
    end

    function [g,dg] = cost(dt,x,u)
        R = 10*eye(2);
        g = u'*R*u;
        dg = [zeros(1,1+size(x,1)),2*u'*R];
    end

    function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;
    end

    function [xtraj] = initTraj(x0,xf,tf0,N)
        % create reasonable middle states
        manipulator_state_m = [-1;pi/2];
        manipulator_state_m2 = [1;pi/2];
        manipulator_vel_m = [5;10];
        manipulator_vel_m2 = [5;0];
        xm = [manipulator_state_m;manipulator_vel_m];
        xm2 = [manipulator_state_m2;manipulator_vel_m2];
        
        % split the positions into three parts from x0 to xm
        N1 = floor(N/3);
        xvec1 = x0*ones(1,N1) + (xm-x0)*linspace(0,1,N1);
        % and xm to xm2
        N2 = N1;
        xvec2 = xm*ones(1,N2) + (xm2-xm)*linspace(0,1,N2);
        % and xm2 to xf
        N3 = N - N1 - N2;
        xvec3 = xm2*ones(1,N3) + (xf-xm2)*linspace(0,1,N3);
        
        % then create the trajectory
        xtraj = PPTrajectory(foh(linspace(0,tf0,N),[xvec1 xvec2 xvec3]));          
    end
    
    function [xtraj,utraj] = improveGuess(xtraj,utraj)
        xtraj = xtraj;
        utraj = utraj;
        %xtraj_noise = awgn(HOW_DO_YOU_GET_THE_POINTS,10,'measured');
        %utraj_noise = awgn(HOW_DO_YOU_GET_THE_POINTS,10,'measured');
    end

    % playback the trajectory
    %v.playback(xtraj,struct('slider',true));
    v.playbackAVI(xtraj,'~/Desktop/terrainTest.swf');
    
    % for simulation need to wrap the plant in a TimeStepping to get contacts
    % plant = TimeSteppingRigidBodyManipulator(plant,.001,options);
    % to get dynamics at a point use the manipulatorDynamics
    % [H,C,B] = plant.manipulatorDynamics(q,qd);
end