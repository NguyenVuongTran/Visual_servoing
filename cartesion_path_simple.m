%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CARTESION PATH PLANNING
%This script is use to control the robot end-effector to follow a
%predefined waypoints
%in this simpltest script, robot will be control from initial position to
%next position near the target object
%the target object is added in Gazebo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Initiate ros node (Matlab Node)
ipaddress = '192.168.0.5';
rosinit(ipaddress,11311);
JointNames = {'joint_1';'joint_2';'joint_3';'joint_4';'joint_5';'joint_6'};
%Link names
LinkName = {'base_link';'shoulder_link';'arm_link';'forearm_link';'lower_wrist_link';'upper_wrist_link';'end_effector_link'};

%Register the ROS service to computer Cartesian path from the waypoints
cartesianPathClient = rossvcclient('/my_gen3_lite/compute_cartesian_path');
wayPointMsg = rosmessage(cartesianPathClient);
wayPointMsg.GroupName = 'arm'
wayPointMsg.LinkName = 'end_effector_link';
wayPointMsg.MaxStep = 0.02; 

%Define orientation constraint for the end-effector
OrientationConstraint = rosmessage('moveit_msgs/OrientationConstraint');
OrientationConstraintsQuat = eul2quat([0 0 0]);
OrientationConstraint.Orientation.X = OrientationConstraintsQuat(1,1);
OrientationConstraint.Orientation.Y = OrientationConstraintsQuat(1,2);
OrientationConstraint.Orientation.Z = OrientationConstraintsQuat(1,3);
OrientationConstraint.Orientation.W = OrientationConstraintsQuat(1,4);
wayPointMsg.PathConstraints.OrientationConstraints = OrientationConstraint;

%Subcribe ROS topic to get state of robot joints
subJointStates = rossubscriber('/my_gen3_lite/joint_states');
resultMsgJointStates = rosmessage(subJointStates);
resultMsgJointStates = receive(subJointStates);
wayPointMsg.StartState.JointState = resultMsgJointStates;

%Get target pose through link_states (gazebo)
%The object is manually added to Gazebo
subGazeboLinkStates = rossubscriber('/gazebo/link_states');
resultGazeboMsgLinkStates = rosmessage(subGazeboLinkStates);
resultGazeboMsgLinkStates = receive(subGazeboLinkStates);

%Sine we know that the object state is stored in element #13 of array
targetObjectPose = resultGazeboMsgLinkStates.Pose(13,1);
targetObjectOrientationQuat = eul2quat([0 pi/2 pi/2]);
targetObjectPose.Orientation.X = targetObjectOrientationQuat(1,1);
targetObjectPose.Orientation.Y = targetObjectOrientationQuat(1,2);
targetObjectPose.Orientation.Z = targetObjectOrientationQuat(1,3);
targetObjectPose.Orientation.W = targetObjectOrientationQuat(1,4);
targetObjectPose.Orientation.W = targetObjectOrientationQuat(1,4);
targetObjectPose.Position.X = targetObjectPose.Position.X - 0.03;
targetObjectPose.Position.Y = targetObjectPose.Position.Y - 0.1;
targetObjectPose.Position.Z = targetObjectPose.Position.Z - 0.03;
wayPointMsg.Waypoints = targetObjectPose;

%Call service and and and get result trajectory
trajResult = call(cartesianPathClient,wayPointMsg);

%Send trajectory to robot(in Gazebo) by publishing trajectory to a topic
[pubTrajGoal,goalMsgTraj] = rospublisher('/my_gen3_lite/gen3_lite_joint_trajectory_controller/follow_joint_trajectory/goal');
goalMsgTraj.Goal.Trajectory.JointNames = JointNames;
goalMsgTraj.Goal.Trajectory = trajResult.Solution.JointTrajectory;
send(pubTrajGoal,goalMsgTraj);

