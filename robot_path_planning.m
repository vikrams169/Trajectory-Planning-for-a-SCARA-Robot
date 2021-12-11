% Constructing the robot from the previously written function
robot = createRobot;
axes = show(robot);
axes.CameraPositionMode = 'auto';

% Choosing the Appropriate Trajectory for the robot to follow based on it's styele & complexity
TrajStyle = 'simple1';
switch TrajStyle
    case 'simple1'
        definedPoints = [0.775 0 0.4; 0.775 0 0.35; 0.775 0 0.25; 0.775 0 0.15]; % Alternate set of wayPoints
        definedPointVels = [0.1 0.1 0.1;0.1 0.1 0.1;0.1 0.1 0.1;0.1 0.1 0.1];
    case 'simple2'
        definedPoints = [0.775 0 0.4; 0.425 0.35 0.4];
        definedPointVels = [0.1 0.1 0.1; 0.1 0.1 0.1];
    case 'complex1'
        definedPoints = [0.775 0 0.4; 0.7 0.1 0.35; 0.6 0.2 0.2; 0.5 0.3 0.1; 0.3 0.4 0.2; 0.04 0.1 0.2];
        definedPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0];
    case 'complex2'
        definedPoints = [0.775 0 0.4; 0.5 -0.3 0.1; 0.6 0.2 0.2; 0.5 0.3 0.1; 0.425 0.35 0.085; 0.04 0.1 0.2];
        definedPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0];        
end

% Making a smooth trajectory to follow from the individual points defined
% earlier
numTotalPoints = size(definedPoints,1);
definedpointTime = 4;
waitTimes = (0:size(definedPoints,1)-1)*definedpointTime;
totalTrajTimes = linspace(0,waitTimes(end),numTotalPoints);
trajectory = cubicpolytraj(definedPoints',waitTimes,totalTrajTimes,'VelocityBoundaryCondition',definedPointVels');
                 
% Plotting the defined Trajectory
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

% Performing Inverse Kinematics to 
% Setting the desired weights while using the inverse kinematics solver (first three weights represent orientation/rotation, and the last three translation)
% We put more weight for rotation & translation along the Z axis because
% that is where most of the motion happens for the SCARA robot in its
% designed configuration
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 1 0.1 0.1 1];
initialguess = robot.homeConfiguration;
for idx = 1:size(trajectory,2)
    transform = trvec2tform(trajectory(:,idx)');
    solution(idx,:) = ik('end_effector',transform,weights,initialguess);
    initialguess = solution(idx,:);
end

% Visualizing the Robot's end-effector moving along the trajectory
title('Scara Robot in a Pick & Drop Trajectory')
for idx = 1:size(trajectory,2)
    show(robot,solution(idx,:),'PreservePlot',false,'Frames','off');
    pause(1)
end

hold off
