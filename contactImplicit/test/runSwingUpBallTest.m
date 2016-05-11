function runSwingUpBallTest()
    % construct the plant with terrain and ball
    options.view = 'right';
    terrainHeight = -2.1;
    options.terrain = RigidBodyFlatTerrain(terrainHeight);
    plant = PlanarRigidBodyManipulator('../urdf/TestPlantBoxHand.urdf',options);
    plant = plant.addRobotFromURDF('../urdf/brick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
    plant = plant.setInputLimits(-40,40);
    plant = plant.compile();
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
    ball_state_0 = [-4;-1;0];
    ball_state_f = [-4;-2;0];
    ball_vel_0 = [0;0;0];
    ball_vel_f = [0;0;10];
    %x0 = [manipulator_state_0;manipulator_vel_0];
    %xf = [manipulator_state_f;manipulator_vel_f];
    x0 = [manipulator_state_0;ball_state_0;manipulator_vel_0;ball_vel_0];
    xf = [manipulator_state_f;ball_state_f;manipulator_vel_f;ball_vel_f];
    
    % timespan and number of colocation points
    tf0 = 4;
    N = 10;
    
    % add in relaxation options and iterations limits
    options.MinorFeasibilityTolerance = 1e-4;
    options.MajorFeasibilityTolerance = 1e-4;
    options.MajorOptimalityTolerance = 5e-5;
    options.MajorIterationsLimit = 1000;
    options.IterationsLimit = 1000000;
    options.SuperbasicsLimit= 10000;
    
    % create the contact implicit trajectory object
    prog = ContactImplicitTrajectoryOptimization(plant,N,[2 6],options);
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
        [traj_init] = improveGuess(xtraj,utraj,ltraj,ljltraj);
        v.playback(xtraj);
    end

    function [g,dg] = cost(dt,x,u)
        R = 10*eye(2);
        g = u'*R*u;
        dg = [zeros(1,1+size(x,1)),2*u'*R];
    end

    function [h,dh] = finalCost(t,x)
        Qf = 0;
        h = t + Qf*x(size(x,1));
        dh = [1,zeros(1,size(x,1)-1),Qf];
        return;
    end

    function [xtraj] = initTraj(x0,xf,tf0,N)
        % create reasonable middle states
        manipulator_state_m = [-1;pi/2];
        manipulator_state_m2 = [1;pi/2];
        manipulator_vel_m = [5;10];
        manipulator_vel_m2 = [5;0];
        ball_state_m = [0;-2;0];
        ball_vel_m = [0;0;0];
        %xm = [manipulator_state_m;manipulator_vel_m];
        %xm2 = [manipulator_state_m2;manipulator_vel_m2];
        xm = [manipulator_state_m;ball_state_m;manipulator_vel_m;ball_vel_m];
        xm2 = [manipulator_state_m2;ball_state_m;manipulator_vel_m2;ball_vel_m];
        
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
    
    function [traj_init] = improveGuess(xtraj,utraj,ltraj,ljltraj)
        traj_init.x = xtraj;
        traj_init.u = utraj;
        traj_init.l = ltraj;
        traj_init.ljl = ljltraj;
        %xtraj_noise = awgn(HOW_DO_YOU_GET_THE_POINTS,10,'measured');
        %utraj_noise = awgn(HOW_DO_YOU_GET_THE_POINTS,10,'measured');
    end

    % playback the trajectory
    %v.playback(xtraj,struct('slider',true));
    v.playbackAVI(xtraj,'~/Desktop/ballTest.avi');

end