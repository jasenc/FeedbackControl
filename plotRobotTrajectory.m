% plotRobotTrajectory(startPose, goalPose, robTraj)
% This functions takes the robot's start and goal positions and
% orientations and the robot's trajectory and plots this on a graph
% startPose - 3x1 vector containing the robot's starting x, y position and
% orientation (theta E [0,2pi])
% goalPose - 3x1 vector containing the robot's goal x, y position and
% orientation (theta E [0,2pi])
% robTraj - Nx3 vector containing the robot's x, y position and orientation
% (theta E [0,2pi]) in each row.  The number of rows, N, denotes the number of
% points used to define the robot's trajectory
function plotRobotTrajectory(startPose, goalPose, robTraj)

% initializes a figure window
figure
hold on

% computing the 2x1 vector representing the robot's initial heading
startDx = cos(startPose(6));
startDy = sin(startPose(6));

% determining the base of the robot
robHeading = [startDx; startDy];
baseEndPt1 = startPose(1:2) + 0.5*[0 1; -1 0]*robHeading./norm(robHeading);
baseEndPt2 = startPose(1:2) - 0.5*[0 1; -1 0]*robHeading./norm(robHeading);

% plots the robots start pose
plot([baseEndPt1(1) baseEndPt2(1)], [baseEndPt1(2) baseEndPt2(2)], 'r')
quiver(startPose(1), startPose(2), startDx, startDy, 1, 'r')

% computing the 2x1 vector representing the robot's final heading
goalDx = cos(goalPose(6));
goalDy = sin(goalPose(6));

% determining the base of the robot
robHeading = [goalDx; goalDy];
baseEndPt1 = goalPose(1:2) + 0.5*[0 1; -1 0]*robHeading./norm(robHeading);
baseEndPt2 = goalPose(1:2) - 0.5*[0 1; -1 0]*robHeading./norm(robHeading);

% plots the robots start pose
plot([baseEndPt1(1) baseEndPt2(1)], [baseEndPt1(2) baseEndPt2(2)], 'b')
quiver(goalPose(1), goalPose(2), goalDx, goalDy, 1, 'b')

% plots the trajectories
plot(robTraj(:,1), robTraj(:,2), 'k')

axis equal
