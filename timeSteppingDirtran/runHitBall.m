function runHitBall()
    % construct the plant with terrain
    options.view = 'right';
    terrainHeight = -2;
    options.terrain = RigidBodyFlatTerrain(terrainHeight);
    plant = PlanarRigidBodyManipulator('../urdf/TwoLinkMostMinimalCollisions.urdf',options);
    plant = plant.setInputLimits(-40,40);
    % add the ball
    plant = plant.addRobotFromURDF('../urdf/ball1point.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
    plant = plant.compile();
    % construct the visualizer
    v = plant.constructVisualizer();
    v.axis = 5*[-1 1 -1 1];
    %v.inspector();
    %return;
    
    % total timespan and number of colocation points
    tmin = 0.5;
    tmax = 1.5;
    tf0 = (tmin+tmax)/2;
    N = 100;
    
    % initialize the trajectory
    [x0,xf,traj_init.x] = initTraj(tf0,N);
    
    % try 10 interations improving guess each time
    for attempts=1:10
        disp('Attempt #');
        disp(attempts);
        
        % compute the trajectory solver
        tic
        prog = computeProblem(plant,x0,xf,N,tmin,tmax,attempts,options);
        toc
        
        % solve
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        
        % if done break
        if info==1, break; end
        
        disp(info)
        info
        break;
        
        % else play trajectory for debug purposes
        v.playback(xtraj);
        
        % else improve the guess
        ltraj = [0];
        ljltraj = [0];
        traj_init = improveGuess(traj_init,xtraj,utraj,ltraj,ljltraj);
    end
    
    % playback the trajectory
    %v.playback(xtraj,struct('slider',true));
    v.playbackAVI(xtraj,'~/Desktop/HitBall.avi');
    
    %{
      %%%%%%% Helper Functions Follow %%%%%%%%
    %}
    
    % initialize the trajectory based on the start and end and middle
    % states as described below
    function [x0,xf,xtraj] = initTraj(tf0,N)
        % set start and end points
        manipulator_state_0 = [-pi/2;0];
        manipulator_state_f = [pi;0];
        manipulator_vel_0 = [0;0];
        manipulator_vel_f = [0;0];
        ball_state_0 = [0;-1;0];
        ball_state_f = [-2;-1.9;0];
        ball_vel_0 = [0;0;0];
        ball_vel_f = [-5;0;0];
        x0 = [manipulator_state_0;ball_state_0;manipulator_vel_0;ball_vel_0];
        xf = [manipulator_state_f;ball_state_f;manipulator_vel_f;ball_vel_f];
        
        % create reasonable middle states
        manipulator_state_m = [-0.2;-0.8];
        manipulator_state_m2 = [0.2;0.8];
        manipulator_vel_m = [5;10];
        manipulator_vel_m2 = [5;10];
        ball_state_m = [0;-1.5;0];
        ball_state_m2 = [-1;-1.9;0];
        ball_vel_m = [0;0;0];
        ball_vel_m2 = [-2;0;-1];
        xm = [manipulator_state_m;ball_state_m;manipulator_vel_m;ball_vel_m];
        xm2 = [manipulator_state_m2;ball_state_m2;manipulator_vel_m2;ball_vel_m2];
        
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
        % use both energy and distance from ground to coax it not to get
        % close to the ground but also distance to ball to coax it to hit
        % the darn ball
        nX = plant.getNumStates();
        nQ = nX/2;
        nQd = nX - nQ;
        q = x(1:nQ);
        kinsol = plant.doKinematics(q);
        hand_body = 3;
        pos_on_hand_body = [0;-1.1];
        [hand_pos,dHand_pos] = plant.forwardKin(kinsol,hand_body,pos_on_hand_body);
        
        R = 10*eye(size(u,1));
        Q1 = -100;
        Q2 = 100;
        
        g = u'*R*u + Q1*hand_pos(2) + Q2*(hand_pos(1) - x(3));
        dgdt = 0;
        dgdx = Q1*[dHand_pos(2,:),zeros(1,nQd)] + Q2*[dHand_pos(1,:),zeros(1,nQd)] - Q2*[0,0,1,0,0,zeros(1,nQd)];
        dgdu = 2*u'*R;
        dg = [dgdt,dgdx,dgdu];
    end

    % final cost
    function [h,dh] = finalCost(t,x)
        Qf = 50;
        h = Qf*t;
        dh = [Qf,zeros(1,size(x,1))];
        return
        
        % short circuit just position in x of ball
        Qf = 500;
        h = Qf*x(3);
        dhdt = 0;
        dhdq = [0,0,Qf,0,0];
        dhdqd = zeros(1,5);
        dh = [dhdt,dhdq,dhdqd];
    end

    % compute the trajectory optimization problem
    function [prog] = computeProblem(plant,x0,xf,N,tmin,tmax,slack_level,options)
        % add in relaxation options and iterations limits start with a lot
        % of slack and then reduce it over the iterations
        offset = -1;
        switch(slack_level+offset)
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
        options.MajorIterationsLimit = 100000;
        options.IterationsLimit = 100000000;
        options.SuperbasicsLimit= 1000000;
        options.integration_method = 1;
        %options.time_option = 1;
        
        % create the trajectory optimization object
        prog = DirtranTrajectoryOptimization(plant,N,[tmin tmax],options);
        
        % add the start and end state
        prog = prog.addStateConstraint(ConstantConstraint(x0),1);
        %prog = prog.addStateConstraint(ConstantConstraint(xf),N);
        
        % add the running and final cost functions
        prog = prog.addRunningCost(@cost);
        prog = prog.addFinalCost(@finalCost);
        
        % add in the joint limit constraints  
        joint_limits = BoundingBoxConstraint(plant.joint_limit_min,plant.joint_limit_max);
        joint_limit_size = size(plant.joint_limit_min,1);
        prog = prog.addStateConstraint(joint_limits,[1:N],[1:joint_limit_size]);
        
        % add in the input constraints
        prog = prog.addInputConstraint(BoundingBoxConstraint(-40*ones(nnz(plant.B),1),40*ones(nnz(plant.B),1)),[1:N]);
    end
end