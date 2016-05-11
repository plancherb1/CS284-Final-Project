## CS284-Final-Project

#### /contactImplicit - initial work with the ContactImplicitTrajectoryOptimization
  &nbsp;&nbsp;##### /swingUp - iterations of the swing up problem using the manipulator urdf
  &nbsp;&nbsp;##### /test - test iterations using the simpler test plant urdf for debug
  &nbsp;&nbsp;##### runCatch.m - partially completed code to have the manipulator urdf catch a moving ball (learned this method fails while working on this)
  &nbsp;&nbsp;##### runLiftBall.m - partially completed code to have the manipulator urdf lift a ball in the air (learned this method fails while working on this)
  
#### /deptricatedOrBroken - old partial implementations or depricated version of code

#### /timeSteppingDirtran - work with TimeSteppingRigidBodyManipulators and Dirtran to get contacts between objects into the optimization
  &nbsp;&nbsp;##### DirtranTrajectoryOptimizationTSRBM.m - code showing how to adjust your DirtranTrajectoryOptimization to get the code to run
  &nbsp;&nbsp;##### dirtranConditionTSRBM.m - takes in a TSRBM and options and outputs the [f,df] condition for dirtran
  &nbsp;&nbsp;##### runHitBall.m - code to have the arm hit the ball to the left
  &nbsp;&nbsp;##### runSwingUp.m - standard swing up problem using the new method
  &nbsp;&nbsp;##### runSwingUpBall.m - swing up code with a ball falling (or staying on ground)
  &nbsp;&nbsp;##### runSwingUpSimple.m - basic swing up code for debugging the new method
  &nbsp;&nbsp;##### testGRadientDiffs.m - test code to make sure my implementations of gradients works

#### /urdf - all urdf files used in the code
