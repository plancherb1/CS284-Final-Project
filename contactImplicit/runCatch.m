function [p,xtraj,utraj,v,x0] = runCatch
    p = PlanarRigidBodyManipulator('../urdf/PlanarManipulator.urdf');
    p = p.addRobotFromURDF('../urdf/ball.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
    p = p.setInputLimits(-40,40);

    N = 31;
    T = 3;

    x0 = [0;0;0;0;0;0;-10;3*5-2-4.5*9.81;0;0;0;0;0;0;0;3;3*9.81-5;0];

    t_init = linspace(0,T,N);

    % Set the initial guess for x and u, should be dim(x) by N 
    % and dim(u) by N per pset3
    % compute a random u
    u_init_vec = randi([-10 10],5,N);
    % function to force our us into the dynamics
    function [xdot] = simplifiedDynamics(t,x)
        index = int64(t/(T/(N-1)) + 1);
        xdot = p.dynamics(t,x,u_init_vec(:,index));
    end
    % forward simulate function to compute the dynamics
    function [xs] = calcXs(x0,ts)
        [t,xs] = ode45(@simplifiedDynamics,ts,x0);
    end
    % get the xs based on our u guess
    ts = [0:T/(N-1):T];
    x_init_vec = calcXs(x0,ts)';

    % initialize the trajectory optimization
    traj_init.x = PPTrajectory(foh(t_init,x_init_vec));
    traj_init.u = PPTrajectory(foh(t_init,u_init_vec));
    traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
    traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);
    traj_opt =  DircolTrajectoryOptimization(p,N,[T/2 T]);

    % add the state constrints
    traj_opt = traj_opt.addFinalCost(@(tt,x) final_state_obj(p,tt,x));
    traj_opt = traj_opt.addRunningCost(@running_cost_fun);
    traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
    catchConstraint = FunctionHandleConstraint([0;0],[0;0],18,@(x) final_state_con(p,x),1);
    traj_opt = traj_opt.addStateConstraint(catchConstraint,N);

    % set the solver
    traj_opt = traj_opt.setSolver('fmincon');
    traj_opt = traj_opt.setSolverOptions('fmincon','Algorithm','sqp');

    % solve
    tic
    [xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init)
    toc

    % visualize
    v = p.constructVisualizer;
    v.axis = [-5 5 -5 5];
    v.playback(xtraj)
    v.drawWrapper(3,xtraj.eval(3))

    xf = xtraj.eval(3);
    [~,dcon] = final_state_con(p,xf);
    
    % helper functions
    function [f,df] = running_cost_fun(h,x,u)
        f = u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2 + u(5)^2;
        df = [0 zeros(1,18) 2*u(1) 2*u(2) 2*u(3) 2*u(4) 2*u(5)];
    end

    function [f,df] = final_state_con(obj,x)
        q = x(1:9);
        qd = x(10:18);
        kinsol = obj.doKinematics(q);

        % body index, so p.body(4) is the hand
        hand_body = 4;

        % center of hand
        pos_on_hand_body = [0;0];

        % Calculate position of the hand in world coordinates
        % the gradient, dHand_pos, is the derivative w.r.t. q
        [hand_pos,dHand_pos] = obj.forwardKin(kinsol,hand_body,pos_on_hand_body);
        
        % Calculate f and the gradient df/dx
        f = q(8:9) - hand_pos;
        df = [[[0,0,0,0,0,0,0,1,0];[0,0,0,0,0,0,0,0,1]] - dHand_pos, zeros(2,9)];
    end

    function [f,df] = final_state_obj(obj,T,x)
        q = x(1:9);
        qd = x(10:18);
        kinsol = obj.doKinematics(q);
        hand_body = 3;
        pos_on_hand_body = [0;-2.1];
        [hand_pos,dHand_pos] = obj.forwardKin(kinsol,hand_body,pos_on_hand_body);

        % then reward for a higher hand position by reducing final cost by a huge
        % factor cause why not make it huge!
        f = -1000*hand_pos(2);
        df = -1000*[0,dHand_pos(2,:),0,0,0,0,0,0,0,0,0];
    end
end