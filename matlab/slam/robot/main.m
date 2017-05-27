%% Test 

clear;close all;clc;

% robot init
pb = startup_robot('192.168.43.154');


% EKF init
idx = [];
mu = [0;0;0];
sigma = 0.01*eye(3);

% 
target = [0.25;0.25];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);



% optional: plotting function coord frame

% TODO: fix motion (speed to power) - ensure that if zero power set than no
% movement. 