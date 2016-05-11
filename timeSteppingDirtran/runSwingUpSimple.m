function runSwingUpSimple()
    % construct the plant
    options.view = 'right';
    plant = PlanarRigidBodyManipulator('../urdf/PlanarManipulatorMinimalCollisions.urdf',options);
    plant = plant.setInputLimits(-40,40);
    % construct the visualizer
    v = plant.constructVisualizer();
    v.axis = 5*[-1 1 -1 1];
    %v.inspector();
    %return;
    
    % total timespan and number of colocation points
    tmin = 2;
    tmax = 6;
    tf0 = (tmin+tmax)/2;
    N = 30;
    
    % initialize the trajectory
    [x0,xf,traj_init.x] = initTraj(tf0,N);
    
    % compute the problem to solve
    %options.time_option = 1;
    options.MinorFeasibilityTolerance = 1e-5;
    options.MajorFeasibilityTolerance = 5e-4;
    options.MajorOptimalityTolerance = 5e-4;
    options.MajorIterationsLimit = 10000;
    options.IterationsLimit = 10000000;
    options.SuperbasicsLimit= 100000;
    options.integration_method = 1;
    prog = DirtranTrajectoryOptimization(plant,N,[tmin tmax],options);
    prog = prog.addStateConstraint(ConstantConstraint(x0),1);
    prog = prog.addStateConstraint(ConstantConstraint(xf),N);
    prog = prog.addRunningCost(@cost);
    prog = prog.addFinalCost(@finalCost);
    
    % try 10 interations improving guess each time
    for attempts=1:25
        disp('Attempt #');
        disp(attempts);
        
        % solve
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        
        % if done break
        if info==1, break; end
        % else play trajectory for debug purposes
        v.playback(xtraj);
        
        % else improve the guess
        traj_init.x = xtraj;
        traj_init.u = utraj;
    end
    
    % playback the trajectory
    v.playback(xtraj,struct('slider',true));
    %v.playbackAVI(xtraj,'~/Desktop/swingUpBall.avi');
    
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
        
        % create reasonable middle states
        manipulator_state_m = [-1;0];
        manipulator_state_m2 = [1/4;0];
        manipulator_vel_m = [5;0];
        manipulator_vel_m2 = [5;0];
        
        x0 = [manipulator_state_0;manipulator_vel_0];
        xm = [manipulator_state_m;manipulator_vel_m];
        xm2 = [manipulator_state_m2;manipulator_vel_m2];
        xf = [manipulator_state_f;manipulator_vel_f];
        
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

    % runnign cost
    function [g,dg] = cost(dt,x,u)
        % short circuit and just use the effort
        R = 10*eye(size(u,1));
        g = u'*R*u;
        dgdt = 0;
        dgdx = zeros(1,size(x,1));
        dgdu = 2*u'*R;
        dg = [dgdt,dgdx,dgdu];
    end

    % final cost
    function [h,dh] = finalCost(t,x)
        h = 10*t;
        dh = [1,zeros(1,size(x,1))];
    end
end