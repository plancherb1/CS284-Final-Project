function [f_residual, df_residual] = testGradientDiffs()
    % Get plants
    PRBM = PlanarRigidBodyManipulator('../urdf/TwoLink.urdf');
    dt = 0.0001;
    TSRBM = TimeSteppingRigidBodyManipulator(PRBM,dt);
    
    % number of rounds
    iterations = 10;
    % TEST FORWARD_EULER (1) vs MIDPOINT (2) vs INTEGRATED_FORWARD_EULER (3)
    % OR 1 as compared to 3 (4) OR 1 as compraed to 2 (5)
    test = 1;
    
    % save residuals
    f_residual = [];
    df_residual = [];
    
    % run the iterations
    for iter=1:iterations

        % Get rand x,u
        nX = PRBM.getNumStates();
        nU = nnz(PRBM.B);
        x0 = 5 * rand(nX,1);
        u0 = 5 * rand(nU,1);
        x1 = 5 * rand(nX,1);
        u1 = 5 * rand(nU,1);

        % get next step and compute error
        if test == 1 || test == 4 || test == 5
            h = dt;
            [xdot,dxdot] = PRBM.dynamics(0,x0,u0);
            fP = x1 - x0 - h*xdot;
            dfP = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];

            [fT,dfT] = dirtranConditionTSRBM(TSRBM,1,x0,x1,u0);
            
            % print for the comparison test
            if test == 4 || test == 5
                disp('forward');
                disp(fT);
                disp(dfT);
            end
        end
        if test == 2 || test == 5
            h = dt;
            [xdot,dxdot] = PRBM.dynamics(0,.5*(x0+x1),.5*(u0+u1));
            fP = x1 - x0 - h*xdot;
            dfP = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) -.5*h*dxdot(:,nX+2:end) -.5*h*dxdot(:,nX+2:end)];

            options.u1 = u1;
            [fT,dfT] = dirtranConditionTSRBM(TSRBM,3,x0,x1,u0,options);
            
            if test test == 5
                disp('midpoint');
                disp(fT);
                disp(dfT);
            end
        end
        if test == 3 || test == 4
            h = 0.0001;
            [xdot,dxdot] = PRBM.dynamics(0,x0,u0);
            fP = x1 - x0 - h*xdot;
            dfP = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];

            steps = h/dt;
            options.steps = steps;
            [fT,dfT] = dirtranConditionTSRBM(TSRBM,4,x0,x1,u0,options);
            
            % print for the comparison test
            if test == 4
                disp('iterated');
                disp(fT);
                disp(dfT);
            end
        end
        
        try
            f_residual = f_residual + (fP - fT);
            df_residual = df_residual + (dfP - dfT);
        catch
            f_residual = (fP - fT);
            df_residual = (dfP - dfT);
        end
    end
    
    % print the average residual
    f_residual = f_residual ./ iterations;
    df_residual = df_residual ./ iterations;
end