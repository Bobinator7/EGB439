%% Test 
%% NOTE: 57 and 27 are off.

clear;close all;clc;

% robot init
pb = startup_robot('192.168.43.154');


% EKF init
idx = [];
mu = [0.2;0.2;0];
sigma = 0.01*eye(3);


% distress call
load('/maps/map7.mat');

disp('rotate 1')
target_angle = deg2rad(90);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

disp('rotate 2')
target_angle = deg2rad(0);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);


disp('translate 1')
target = [0.5;0.5];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% 
disp('rotate 3')
target_angle = deg2rad(-90);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

target_angle = deg2rad(180);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

hold on; plot_robot(mu,sigma); hold on; plot_beacons(mu,sigma,idx)

disp('translate 2')
target = [1;1];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

hold on; plot_robot(mu,sigma); hold on; plot_beacons(mu,sigma,idx)
% 
disp('rotate 4')
target_angle = deg2rad(170); 
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

target_angle = deg2rad(-120);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

hold on; plot_robot(mu,sigma); hold on; plot_beacons(mu,sigma,idx)
% 
if (size(idx,1)==5)
    disp('5 beacons found - 1st time')
    distCallLoc = getDistressCall(map,mu,idx);
    disp('moving to distress call')
    [mu,sigma,idx] = toPoint(pb, distCallLoc, mu, sigma, idx);

    hold on; plot(distCallLoc(1,1),distCallLoc(2,1),'yx')
    figure;plot_robot(mu,sigma);hold on; plot_beacons(mu,sigma,idx);hold on;plot(distCallLoc(1,1),distCallLoc(2,1),'y*');title('Final Plot')
    hold on;plot(realLoc.map(3:end,2),realLoc.map(3:end,3),'x');
    return
end

disp('shit did not find it')


% target = [0;0];
% [mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% optional: plotting function coord frame

% TODO: adjust range
% TOD): less trust motion mmore camera
% TODO: fix scale
% TODO: clean up graph
% TODO: FINISH!!!!!!!

