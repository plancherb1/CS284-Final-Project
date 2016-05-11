# CS284-Final-Project

###/contactImplicit - initial work with the ContactImplicitTrajectoryOptimization
  /swingUp - iterations of the swing up problem using the manipulator urdf
  /test - test iterations using the simpler test plant urdf for debug
  runCatch.m - partially completed code to have the manipulator urdf catch a moving ball (learned this method fails while working on this)
  runLiftBall.m - partially completed code to have the manipulator urdf lift a ball in the air (learned this method fails while working on this)
  
###/deptricatedOrBroken - old partial implementations or depricated version of code

###/timeSteppingDirtran - work with TimeSteppingRigidBodyManipulators and Dirtran to get contacts between objects into the optimization 
  DirtranTrajectoryOptimizationTSRBM.m - code showing how to adjust your DirtranTrajectoryOptimization to get the code to run
  dirtranConditionTSRBM.m - takes in a TSRBM and options and outputs the [f,df] condition for dirtran
  runHitBall.m - code to have the arm hit the ball to the left
  runSwingUp.m - standard swing up problem using the new method
  runSwingUpBall.m - swing up code with a ball falling (or staying on ground)
  runSwingUpSimple.m - basic swing up code for debugging the new method
  testGRadientDiffs.m - test code to make sure my implementations of gradients works

###/urdf - all urdf files used in the code
