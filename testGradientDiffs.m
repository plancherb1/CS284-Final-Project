function testGradientDiffs()
    PRBM = PlanarRigidBodyManipulator('../acrobot/acrobot.urdf');
    dt = 0.0001;
    TSRBM = TimeSteppingRigidBodyManipulator(PRBM,dt);
    x = 5 * rand(PRBM.num_positions+PRBM.num_velocities,1);
    u = 5 * rand(nnz(PRBM.B),1);
    [Pxdot,Pdxdot] = PRBM.dynamics(0,x,u)
    [Txdot, Tdxdot] = TSRBM.update(0,x,u)
    norm(Pxdot - Txdot)
    norm(Pdxdot - Tdxdot)
end

    %{
    x = [0;0;0;0];
    u = 5;
    TSRBM1 = TimeSteppingRigidBodyManipulator(PRBM,1);
    [Txdot1, Tdxdot1] = TSRBM1.update(0,x,u);
    Txdotf = [Txdot(1:2);Txdot1(3:4)];
    Tdxdotf = [zeros(size(x,1)/2,size(Tdxdot,2));Tdxdot1(size(x,1)/2+1:end,:)] - Tdxdot;
    toFlip = Tdxdotf(1:size(x,1)/2,2:size(x,1)+1);
    toFlip = -1 * [toFlip(:,size(toFlip,2)/2+1:end),toFlip(:,1:size(toFlip,2)/2)];
    Tdxdotf(1:size(x,1)/2,2:size(x,1)+1) = toFlip;
    norm(Pxdot - Txdotf)
    norm(Pdxdot - Tdxdotf)
    %}