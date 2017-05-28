%% Test 

clear;close all;clc;

% robot init
pb = startup_robot('192.168.43.154');


% EKF init
idx = [];
mu = [0;0;0];
sigma = 0.01*eye(3);


target = [0.5;0.5];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);
% 
target_angle = deg2rad(180); 
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);
target_angle = deg2rad(0);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);
% 
target = [0;0];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% target = [0;0];
% [mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% optional: plotting function coord frame

% TODO: adjust params for motion model

