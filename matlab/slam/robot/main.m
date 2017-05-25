%% Test 

clear;close all;clc;

% robot init
pb = startup_robot('192.168.43.154');


% EKF init
idx = [];
mu = [0;0;0];
sigma = 0.1*eye(3);

% 
target = [0.25;0.25];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% TODO: range/bearing
% TODO: Q/R matrix
% TODO: plotting function coord frame
% TODO: 41 beacon?!?!