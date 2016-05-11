function [f,df] = dirtranConditionTSRBM(TSRBM,method,x0,x1,u0,options)
    
    % extract options
    if nargin < 6
        options = 0;
    else
        if isfield(options,'u1')
            u1 = options.u1;
        end
        if isfield(options,'steps')
            steps = options.steps;
        end
    end
    
    % get the timestep and nX
    dt = TSRBM.timestep;
    nX = size(x0,1);
    
    % get the appropriate f and df
    switch(method)
        case 1 % Forward Euler
            [xk, xkdot] = nextState(TSRBM,x0,u0,nX);
            f = x1 - x0 - (xk - x0);
            df = [-(xk - x0)/dt -xkdot(:,2:1+nX) eye(nX) -xkdot(:,nX+2:end)];
        case 2 % Backward Euler
            error('Drake:dirtranConditionTSRBM:InvalidArgument','Backwards Euler not implemented yet');
        case 3 % Midpoint Euler
            [xk, xkdot] = nextState(TSRBM,.5*(x0+x1),.5*(u0+u1),nX);
            f = x1 - x0 - (xk - .5*(x0+x1));
            df = [-(xk - .5*(x0+x1))/dt -xkdot(:,2:1+nX) (2*eye(nX)-xkdot(:,2:1+nX)) -xkdot(:,nX+2:end) -xkdot(:,nX+2:end)];
        case 4 % Iterated Forward Euler
            xc = x0;
            for attempts=1:steps
                [xk, xkdot] = nextState(TSRBM,xc,u0,nX);
                if attempts == 1
                    df = [-(xk - xc)/dt -xkdot(:,2:1+nX) eye(nX) -xkdot(:,nX+2:end)];
                else
                    df(:,2:1+nX) = df(:,2:1+nX) + -xkdot(:,2:1+nX);
                    df(:,end-size(-xkdot(:,nX+2:end),2)+1:end) = df(:,end-size(-xkdot(:,nX+2:end),2)+1:end) +  -xkdot(:,nX+2:end);
                end
                xc = xk;
            end
            f = x1 - x0 - (xk - x0);
            temp = df(:,2:1+nX);
            n = size(temp,1);
            for index = 1:n
                temp(index,index) = temp(index,index) ./ steps;
            end
            df(:,2:1+nX) = temp;            
    end
end

% include penalty state if the update fails
function [xk, xkdot] = nextState(TSRBM,x,u,nX)
    try
        [xk, xkdot] = TSRBM.update(0,x,u);
    catch
        xk = 1000*rand(nX,1);
        xkdot = [zeros(nX,1) 1000*diag(rand(1,nX)) [zeros(nX/2,2);1000*rand(nX/2,2)]];
    end
end
