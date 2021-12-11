function [robot,homeConfig] = createRobot

%Creating the robot's rigid body tree structure from the URDF model
robot = importrobot('/home/vikrams169/Desktop/scara_modified.urdf');

% Adding gravity presence to the system
gravityVec = [0 0 -9.80665];
robot.Gravity = gravityVec;

% Adding a new, hypothetical 'pick up point' to just beneath the last
% prismatic link in the robot, which would eventually trace the trajectory
offset = 0.02;
newPoint = robotics.RigidBody('pick_up_point');
newPoint.Mass = 0;
newPoint.Inertia = [0 0 0 0 0 0];
setFixedTransform(newPoint.Joint,trvec2tform([offset 0 0]));
addBody(robot,newPoint,'link3');

end


