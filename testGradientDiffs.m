function testGradientDiffs()
    % Get plants
    PRBM = PlanarRigidBodyManipulator('urdf/acrobotTest.urdf');
    dt = 0.0001;
    TSRBM = TimeSteppingRigidBodyManipulator(PRBM,dt);
    
    % Get rand x,u
    nX = PRBM.getNumStates();
    nU = nnz(PRBM.B);
    x0 = 5 * rand(nX,1);
    u0 = 5 * rand(nU,1);
    x1 = 5 * rand(nX,1);
    u1 = 5 * rand(nU,1);

    % TEST FORWARD_EULER (1) vs MIDPOINT (2) vs INTEGRATED_FORWARD_EULER (3) %
    test = 3;
    
    % get next step and compute error
    if test == 1
        h = dt;
        [xdot,dxdot] = PRBM.dynamics(0,x0,u0);
        fP = x1 - x0 - h*xdot;
        dfP = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
        
        [xk, xkdot] = TSRBM.update(0,x0,u0);
        fT = x1 - x0 - (xk - x0);
        dfT = [-(xk - x0)/dt -xkdot(:,2:1+nX) eye(nX) -xkdot(:,nX+2:end)];
    end
    if test == 2
        h = dt;
        [xdot,dxdot] = PRBM.dynamics(0,.5*(x0+x1),.5*(u0+u1));
        fP = x1 - x0 - h*xdot;
        dfP = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
    
        [xk, xkdot] = TSRBM.update(0,.5*(x0+x1),.5*(u0+u1));
        fT = x1 - x0 - (xk - .5*(x0+x1));
        dfT = [-(xk - .5*(x0+x1))/dt -xkdot(:,2:1+nX) eye(nX) -xkdot(:,nX+2:end)];
    end
    if test == 3
        h = 0.01;
        [xdot,dxdot] = PRBM.dynamics(0,x0,u0);
        fP = x1 - x0 - h*xdot;
        dfP = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
        
        steps = h/dt;
        xc = x0;
        for attempts=1:steps
            [xk, xkdot] = TSRBM.update(0,xc,u0);
            try
                dfT(:,2:1+nX) = dfT(:,2:1+nX) * -xkdot(:,2:1+nX); % SOLVE THIS!!!!!
                dfT(:,end-size(-xkdot(:,nX+2:end),2)+1:end) = dfT(:,end-size(-xkdot(:,nX+2:end),2)+1:end) +  -xkdot(:,nX+2:end);
            catch
                dfT = [-(xk - xc)/dt -xkdot(:,2:1+nX) eye(nX) -xkdot(:,nX+2:end)];
            end
            xc = xk;
        end
        fT = x1 - x0 - (xk - x0);
        %dfT(:,2:1+nX) = dfT(:,2:1+nX) ./ steps;
    end
    
    fP - fT
    dfP - dfT
    dfP
    dfT
end