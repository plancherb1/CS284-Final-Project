function runSwingUpWithBall()
    % construct the plant with terrain
    options.view = 'right';
    terrainHeight = -2.4;
    options.terrain = RigidBodyFlatTerrain(terrainHeight);
    plant = PlanarRigidBodyManipulator('../urdf/PlanarManipulator.urdf',options);
    % add the ball
    plant = plant.addRobotFromURDF('../urdf/PlanarManipulatorBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
    plant = plant.setInputLimits(-40,40);
    plant = plant.compile();
    % construct the visualizer
    v = plant.constructVisualizer();
    v.axis = 5*[-1 1 -1 1];
    %v.inspector();
    %return;
    
    % timespan and number of colocation points
    tf0 = 4;
    N = 10;
    
    % initialize the trajectory
    [x0,xf,traj_init.x] = initTraj(tf0,N);
    
    % try 10 interations improving guess each time
    for attempts=1:10
        disp('Attempt #');
        disp(attempts);
        tic
        % compute the trajectory solver
        prog = computeProblem(plant,x0,xf,N,i,options);
        % solve
        [xtraj,utraj,ltraj,ljltraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        % if done break
        if info==1, break; end
        % else improve the guess
        traj_init = improveGuess(traj_init,xtraj,utraj,ltraj,ljltraj);
        % update the slack options
        %options = calcOptions(i);
        v.playback(xtraj);
    end
    
    % playback the trajectory
    %v.playback(xtraj,struct('slider',true));
    v.playbackAVI(xtraj,'~/Desktop/swingUpBall.avi');
    % for simulation need to wrap the plant in a TimeStepping to get contacts
    % plant = TimeSteppingRigidBodyManipulator(plant,.001,options);
    % to get dynamics at a point use the manipulatorDynamics
    % [H,C,B] = plant.manipulatorDynamics(q,qd);
    
    %{
      %%%%%%% Helper Functions Follow %%%%%%%%
    %}
    
    % initialize the trajectory based on the start and end and middle
    % states as described below
    function [x0,xf,xtraj] = initTraj(tf0,N)
        % set start and end points
        manipulator_state_0 = [-pi/2;0;0;0;0;0];
        manipulator_state_f = [pi;0;0;0;0;0];
        manipulator_vel_0 = zeros(6,1);
        manipulator_vel_f = zeros(6,1);
        %x0 = [manipulator_state_0;manipulator_vel_0];
        %xf = [manipulator_state_f;manipulator_vel_f];
        brick_state_0 = [3;-1.3;0];
        brick_state_f = [3;-2.3;0];
        brick_vel_0 = [0;0;0];
        brick_vel_f = [0;0;0];
        x0 = [manipulator_state_0;brick_state_0;manipulator_vel_0;brick_vel_0];
        xf = [manipulator_state_f;brick_state_f;manipulator_vel_f;brick_vel_f];
        
        % create reasonable middle states
        manipulator_state_m = [-1;pi/2;pi/2;-pi/2;-pi/2;pi/2];
        manipulator_state_m2 = [1;pi/2;pi/2;-pi/2;-pi/2;pi/2];
        manipulator_vel_m = [5;10;0;0;0;0];
        manipulator_vel_m2 = [5;0;0;0;0;0];
        %xm = [manipulator_state_m;manipulator_vel_m];
        %xm2 = [manipulator_state_m2;manipulator_vel_m2];
        brick_state_m = [0;-2;0];
        brick_vel_m = [0;0;0];
        xm = [manipulator_state_m;brick_state_m;manipulator_vel_m;brick_vel_m];
        xm2 = [manipulator_state_m2;brick_state_m;manipulator_vel_m2;brick_vel_m];
        
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
    function [traj_init] = improveGuess(traj_init,xtraj,utraj,ltraj,ljltraj)
        traj_init.x = xtraj;
        traj_init.u = utraj;
        %traj_init.l = ltraj;
        %traj_init.ljl = ljltraj;
    end

    % runnign cost
    function [g,dg] = cost(dt,x,u)
        % short circuit and just use the effort
        %R = 100*eye(6);
        %g = u'*R*u;
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        %return
        
        % cost matricies penalize more for moving fingers and deviation of
        % the ball from the goal
        Q = diag([ones(6,1)',ones(3,1)',ones(6,1)',ones(3,1)']);
        R = diag([5,5,5,5,5,5]);
        
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

    % final cost
    function [h,dh] = finalCost(t,x)
        %h = t;
        %dh = [1,zeros(1,size(x,1))];
        %return;
        
        xerr = x-xf;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        xerr(2,:) = mod(xerr(2,:)+pi,2*pi)-pi;
        xerr(3,:) = mod(xerr(3,:)+pi,2*pi)-pi;
        xerr(4,:) = mod(xerr(4,:)+pi,2*pi)-pi;
        xerr(5,:) = mod(xerr(5,:)+pi,2*pi)-pi;
        
        Qf = diag([10*ones(6,1)',10*ones(3,1)',ones(6,1)',ones(3,1)']);
        a = 10;
        
        h = a*t + xerr'*Qf*xerr;
        dh = [a, 2*xerr'*Qf];
    end

    % compute the trajectory optimization problem
    function [prog] = computeProblem(plant,x0,xf,N,i,options)
        % add in relaxation options and iterations limits
        switch(i)
            case 1
                minor = 1;
                major = 0.5;
            case 2
                minor = 1e-1;
                major = 5e-2;
            case 3
                minor = 1e-2;
                major = 5e-3;
            case 4
                minor = 1e-3;
                major = 5e-4;
            otherwise
                minor = 1e-4;
                major = 5e-5;
        end
        options.MinorFeasibilityTolerance = minor;
        options.MajorFeasibilityTolerance = major;
        options.MajorOptimalityTolerance = major;
        options.compl_slack = minor;
        options.lincompl_slack = major;
        options.jlcompl_slack = major;
        options.MajorIterationsLimit = 10000;
        options.IterationsLimit = 10000000;
        options.SuperbasicsLimit= 100000;
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
    end
end