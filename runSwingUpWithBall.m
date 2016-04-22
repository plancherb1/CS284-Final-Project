function runSwingUpWithBall()
    % construct the plant
    plant = PlanarRigidBodyManipulator('PlanarManipulatorMinimalCollisions.urdf');
    % add the ball and the stand for the ball
    %plant = plant.addRobotFromURDF('ball.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
    %plant = plant.addRobotFromURDF('PlanarManipulatorBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
    plant = plant.addRobotFromURDF('PlanarManipulatorStand.urdf');
    plant = plant.setInputLimits(-40,40);
    plant = plant.compile();
    % construct the visualizer
    v = plant.constructVisualizer();
    v.axis = 5*[-1 1 -1 1];
    v.inspector();
    return;
    
    % set start and end points
    manipulator_state_0 = [-pi/2;0;0;0;0;0];
    manipulator_state_f = [pi;0;0;0;0;0];
    manipulator_vel_0 = zeros(6,1);
    manipulator_vel_f = zeros(6,1);
    x0 = [manipulator_state_0;manipulator_vel_0];
    xf = [manipulator_state_f;manipulator_vel_f];
    %brick_state_0 = [0;-2;0];
    %brick_state_f = [0;-2;0];
    %brick_vel_0 = [0;0;0];
    %brick_vel_f = [0;0;0];
    %x0 = [manipulator_state_0;brick_state_0;manipulator_vel_0;brick_vel_0];
    %xf = [manipulator_state_f;brick_state_f;manipulator_vel_f;brick_vel_f];
    
    % timespan and number of colocation points
    tf0 = 4;
    N = 50;
    
    % add in relaxation options and iterations limits
    options.MinorFeasibilityTolerance = 1e-3;
    options.MajorFeasibilityTolerance = 1e-3;
    options.MajorOptimalityTolerance = 5e-4;
    options.MajorIterationsLimit = 1000;
    options.IterationsLimit = 1000000;
    options.SuperbasicsLimit= 10000;
    
    % create the contact implicit trajectory object with relaxation options
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
        [traj_init.x, traj_init.u] = improveGuess(xtraj,utraj);
        v.playback(xtraj);
    end

    function [g,dg] = cost(dt,x,u)
        % short circuit and just use the effort
        R = 100*eye(6);
        g = u'*R*u;
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return
        
        % cost matricies penalize more for moving fingers and deviation of
        % the ball from the goal
        Q = diag([ones(6,1)',10*ones(3,1)',zeros(6,1)',zeros(3,1)']);
        R = diag([1,1,5,5,10,10]);
        
        % penalize for deviation in x as well
        xerr = x-xf;
        % wrap angles
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        % compute total cost
        g = xerr'*Q*xerr + u'*R*u;

        % and the derivative
        dgddt = 0;
        dgdx = 2*xerr'*Q;
        dgdu = 2*u'*R;
        dg = [dgddt,dgdx,dgdu];
    end

    function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;

        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;

        Qf = 100*diag([10,10,1,1]);
        h = sum((Qf*xerr).*xerr,1);
        dh = [0, 2*xerr'*Qf];
    end

    function [xtraj] = initTraj(x0,xf,tf0,N)
        % create reasonable middle states
        manipulator_state_m = [-1;pi/2;pi/2;-pi/2;-pi/2;pi/2];
        manipulator_state_m2 = [1;pi/2;pi/2;-pi/2;-pi/2;pi/2];
        manipulator_vel_m = [5;10;0;0;0;0];
        manipulator_vel_m2 = [5;0;0;0;0;0];
        xm = [manipulator_state_m;manipulator_vel_m];
        xm2 = [manipulator_state_m2;manipulator_vel_m2];
        %brick_state_m = [0;-2;0];
        %brick_vel_m = [0;0;0];
        %xm = [manipulator_state_m;brick_state_m;manipulator_vel_m;brick_vel_m];
        %xm2 = [manipulator_state_m2;brick_state_m;manipulator_vel_m2;brick_vel_m];
        
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
    
    % would be better if we added gaussian noise or something to the
    % previous trajectory so that we don't get stuck in a local min even if
    % it is failing
    function [xtraj,utraj] = improveGuess(xtraj,utraj)
        xtraj = xtraj;
        utraj = utraj;
    end

    % playback the trajectory
    v.playback(xtraj,struct('slider',true));
    
    % for simulation need to wrap the plant in a TimeStepping to get contacts
    % plant = TimeSteppingRigidBodyManipulator(plant,.001,options);
    % to get dynamics at a point use the manipulatorDynamics
    % [H,C,B] = plant.manipulatorDynamics(q,qd);
end