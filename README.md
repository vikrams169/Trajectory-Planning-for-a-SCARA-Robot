# Trajectory-Planning-for-a-SCARA-Robot
A demonstration for a SCARA robot to follow a pre-defined pick and place trajectory along its end-effector using concepts of inverse kinematics in MATLAB (as a part of the robotics toolbox)<br><br>
<h3>Designing the SCARA Robot</h3>
The SCARA robot configuration is pretty standard for the most part. After looking at a standard set of realistic dimensions and joint types online, I constructed a ‘urdf’ file for representing the robot as a ‘RigidBodyTree’ structure in the ‘Robotics Toolbox’ of Matlab. This was done by hand-constructing and editing an XML file structure of ‘urdf’, with the help of ROS. The final robot structure looks as shown below.
<img src="/images/scara_robot.png">
Each joint has its own frame attached to it, along with the global frame (not shown in the picture). In addition, the final frame attached below the prismatic joint’s sliding link represents the point (at its origin) where an object would be expected to be picked up/placed. The exact position of this point is at an offset of 0.2 m from the end of the prismatic joint’s sliding link. This point would be the one that would eventually trace out the defined trajectories for the pick/place experiment.<br><br>
<h3>Trajectories Tested</h3>
In the robot’s home configuration, the ‘pick and place’ point positioned at an offset from the main robot mentioned previously has coordinates of (0.775,0,0.4). Four different pick and place trajectories starting from this point are analysed to understand the robot’s inverse kinematics. They look like:
<ul><li><b>Simple Trajectory 1</b><br>
  This trajectory consists of the most simple set of orientations possible, where the robot just has to move its sliding link from the prismatic joint up and down, along the Z-axis. It would not require any further motion from any other joint or link to move along the trajectory and fulfil its pick and place objective.
  <img src="/images/simple1.png">
  <li><b>Simple Trajectory 2</b><br>
    This trajectory is also a very basic trajectory like the previous one, except that this trajectory does not involve movement of the prismatic joint’s sliding link since the Z coordinate remains the same throughout the trajectory. The movement of the end effector along the trajectory would solely be due to the rotation of the 2 links about their respective revolute joints along the Z-axis.
    <img src="/images/simple2.png">
    <li><b>Complex Trajectory 1</b><br>
    The first complex trajectory as shown below is a step more complex where the movement of the sliding link of the prismatic joint, as well as rotation about each of the revolute joints, is needed to trace the highlighted trajectory. However, the trajectory is still more or less straightforward without requiring any abrupt/hasty movements as such.
    <img src="/images/complex1.png">
      <li><b>Complex Trajectory 2</b><br>
    This trajectory is similar to the previous one except for the fact that it has more abrupt and sharp turns and twists, which makes it slightly more difficult for the inverse kinematics solver to find a smooth solution set for those specific turning points.
    <img src="/images/complex2.png">
        <h3>Analysis</h3>
        One of the parameters while using the inverse kinematics solver in the ‘Robotics Toolbox’ of Matlab, is the ‘weights’ matrix, which corresponds to an input of 6 parameters, the first three representing orientation along the three axes, and the last three representing the translation along the three axes. Playing around with these weights doesn’t yield much difference in tracing the trajectory in the first two simple trajectories, but subtle differences can be noticed in tracing the two complex trajectories. These weights help the inverse kinematics solver make decisions in what to give preference to while solving the robot’s movements for the trajectory. Since the SCARA robot used itself has only three degrees of freedom, assigning three of these weights as zero is perfectly alright, and even desirable in some cases when dealing with complex trajectories, as putting unnecessary weights for certain movements limits the solution space.<br><br>
When playing around with these weights in the two complex trajectories, it was observed that it was favourable to keep the weights for orientation and translation along with the Z-axis high since both the revolute joints, as well as the prismatic joints have their axes aligned along the Z-axis. On keeping these two sets of weights low, and the other four high, we observe slight problems with the last two trajectories, as would be expected.<br><br>
In addition to weights, another parameter was highly played around with. When one generates the trajectory, the end effector doesn’t exactly trace the entire spline of the trajectory, but rather just a set of specified points along the spline, the number of such points which can be changed in the code. On increasing this number to resemble the trajectory more closely, tracing the complex trajectory sometimes fails. This is because some set adjacent points to trace are often very hard for the inverse kinematic solver to fulfil both at once. This however again is not a problem in either of the simple trajectories since the orientation of all the points in the spline, whether large in number or not is more or less fixed, or highly similar.<br><br>
The code for creating the SCARA robot as a 'rigid body tree' from the 'urdf' file in matalab (in program 'create_robot.m'), and then to test it on the trajectory (in program 'robot_path_planning.m') are based off the basic syntac provided from MATLAB's official documentation for it's robotics toolbox, the link to which is available <a href="https://in.mathworks.com/matlabcentral/fileexchange/65316-designing-robot-manipulator-algorithms">here</a>. Further, the SCARA robot's 'urdf' file used is based off the xacro files used by Katharine Conroy <a href="https://github.com/KatConroy57/scara">here</a> (amazing work by the way)!<br><br>
This project was done as a part of the course Multi Body Dynamics (ME518) under the guidance of Dr. Manish Agarwal, at IIT Ropar.
