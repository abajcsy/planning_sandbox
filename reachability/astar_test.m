%% Test AStar
clc
clf
clear all

% Setup environment bounds.
lowEnv = [0;0];
upEnv = [10;7];

% Setup discrete world size.
simWidth = 20;
simHeight = 14;

% Create planner.
planner = AStar(lowEnv, upEnv, simWidth, simHeight);

% Initial condition.
start = [2,2];
goal = [19,2];

% Generate A* plan from start to goal.
waypts = planner.plan(start, goal);

% Setup obstacle.
lowRealObs = [4;1];
upRealObs = [7;4];

% Plotting.
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs);
plt.plotWaypts(waypts, simWidth, simHeight);

