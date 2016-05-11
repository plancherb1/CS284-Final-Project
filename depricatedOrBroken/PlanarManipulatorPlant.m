classdef PlanarManipulatorPlant < Manipulator 
  
  properties
    % physical parameters
    %{ 
    l1 = 1; l2 = 2;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=.1;  b2=.1;
    lc1 = .5; lc2 = 1; 
    Ic1 = .083;  Ic2 = .33;
    uG;
    xG;
    %}
  end
  
  methods
    function obj = PlanarManipulatorPlant
      obj = obj@Manipulator(6,5,6)
      obj = PlanarRigidBodyManipulator('PlanarManipulator.urdf')
      %{
      obj = setInputLimits(obj,-10,10);
      obj = setInputFrame(obj,CoordinateFrame('PlanarManipulatorInput',5,'u',{'tau1','tau2','tau3','tau4','tau5'}));
      obj = setStateFrame(obj,CoordinateFrame('PlanarManipulatorState',12,'x',{'theta1','theta2','theta3','theta4','theta5','theta6','theta1dot','theta2dot','theta3dot','theta4dot','theta5dot','theta6dot'}));
      obj = setOutputFrame(obj,obj.getStateFrame);
      
      obj.xG = Point(obj.getStateFrame,[pi;0;0;0;0;0;0;0;0;0;0;0]);
      obj.uG = Point(obj.getInputFrame,[0;0;0;0;0]);
      
      obj = setParamFrame(obj,CoordinateFrame('PlanarManipulatorParams',6,'p',...
       { 'b1','b2','lc1','lc2','Ic1','Ic2' }));
      obj = setParamLimits(obj,zeros(obj.getParamFrame.dim,1));
      %}
    end

    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      [H,C,B] = obj.manipulatorDynamics(q,qd);
    end
 %{   
    function [T,U] = energy(obj,x)
      m1=obj.m1; m2=obj.m2; l1=obj.l1; g=obj.g; lc1=obj.lc1; lc2=obj.lc2; b1=obj.b1; b2=obj.b2;
      I1 = obj.Ic1 + obj.m1*obj.lc1^2; I2 = obj.Ic2 + obj.m2*obj.lc2^2;
      q = x(1:2); qd = x(3:4);
      c = cos(q(1:2,:));  s = sin(q(1:2,:));  c12 = cos(q(1,:)+q(2,:));
      
      T = .5*I1*qd(1)^2 + .5*(m2*l1^2 + I2 + 2*m2*l1*lc2*c(2))*qd(1)^2 + .5*I2*qd(2)^2 + (I2 + m2*l1*lc2*c(2))*qd(1)*qd(2);
      U = -m1*g*lc1*c(1) - m2*g*(l1*c(1)+lc2*c12);
    end
    
    function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x = getInitialState(obj)
      x = .1*randn(12,1);
    end
    
    function n = getNumPositions(obj)
      n = 6;
    end
    
    function n = getNumVelocities(obj)
      n = 6;
    end
    
    function [c,V]=balanceLQR(obj)
      Q = diag([10,10,1,1]); R = 1;
      if (nargout<2)
        c = tilqr(obj,obj.xG,obj.uG,Q,R);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V] = tilqr(obj,obj.xG,obj.uG,Q,R);
        pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
      end
    end
    
    function [utraj,xtraj]=swingUpTrajectory(obj)
      x0 = zeros(4,1); 
      xf = double(obj.xG);
      tf0 = 4;
      
      N = 21;
      prog = DircolTrajectoryOptimization(obj,N,[2 6]);
      prog = prog.addStateConstraint(ConstantConstraint(x0),1);
      prog = prog.addStateConstraint(ConstantConstraint(xf),N);
      prog = prog.addRunningCost(@cost);
      prog = prog.addFinalCost(@finalCost);
      
      traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
      
      for attempts=1:10
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        if info==1, break; end
      end

      function [g,dg] = cost(dt,x,u)
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,10,1,1]);
        R = 100;
        g = sum((Q*xerr).*xerr + (R*u).*u,1);
        
        if (nargout>1)
          dgddt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg = [dgddt,dgdx,dgdu];
        end
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
        
        if (nargout>1)
          dh = [0, 2*xerr'*Qf];
        end
      end      

    end

%}    
  end
  
end
