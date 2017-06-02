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
if (size(idx,1)==5)
    distLoc = getDistressCall(map1,mu,idx);
    [mu,sigma,idx] = toPoint(pb, distLoc, mu, sigma, idx);
    return
end

target = [0.75;0.25];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);
target_angle = deg2rad(180); 
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);
target_angle = deg2rad(0);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

if (size(idx,1)==5)
    distLoc = getDistressCall(map1,mu,idx);
    [mu,sigma,idx] = toPoint(pb, distLoc, mu, sigma, idx);
    return
end



% target = [0;0];
% [mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% optional: plotting function coord frame

% TODO: adjust params for motion model

