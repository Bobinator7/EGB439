%% Test 
%% NOTE: 57 and 27 are off.
tic;
clear;close all;clc;

% robot init
pb = startup_robot('192.168.43.154');


% EKF init
idx = [];
mu = [0.2;0.2;0];
sigma = 0.01*eye(3);


% distress call
load('map1.mat'); % map1 within 30cm each time(3/3). % map2 within (1/3).  %map3 within 30cm (1/2) %map4 within (2/2)
%map rotated - within 30cm, beacons not in bound

disp('rotate 1')
target_angle = deg2rad(90);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

disp('rotate 2')
target_angle = deg2rad(-30);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);


disp('translate 1')
target = [1;1];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% 
disp('rotate 3')
target_angle = deg2rad(-145);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

target_angle = deg2rad(170);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

%hold on; plot_robot(mu,sigma); hold on; plot_beacons(mu,sigma,idx)
% 
% disp('translate 2')
% target = [1;1];
% [mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);
% 
% hold on; plot_robot(mu,sigma); hold on; plot_beacons(mu,sigma,idx)
% % 
% disp('rotate 4')
% target_angle = deg2rad(170); 
% [mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);
% 
% target_angle = deg2rad(-120);
% [mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);
% 
% hold on; plot_robot(mu,sigma); hold on; plot_beacons(mu,sigma,idx)
% 
if (size(idx,1)==5)
    disp('5 beacons found - 1st time')
    %map = [1,0.5,0.5;27,1.06,0.125;29,1.775,0.695];
    distCallLoc = getDistressCall(map,mu,idx);
    disp('moving to distress call')
    [mu,sigma,idx] = toPointDistress(pb, distCallLoc, mu, sigma, idx);

    hold on; plot(distCallLoc(1,1),distCallLoc(1,2),'yx')
    figure;plot_robot(mu,sigma);hold on; plot_beacons(mu,sigma,idx);hold on;plot(distCallLoc(1,1),distCallLoc(1,2),'y*');title('Final Plot')
    realLoc = load('/maps/map.mat');
    hold on;plot(realLoc.map(3:end,2),realLoc.map(3:end,3),'x');
    toc;
    return
end

disp('shit did not find it. Try again!')

disp('translate 1')
target = [1.25;1.25];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);

% 
disp('rotate 3')
target_angle = deg2rad(-145);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

target_angle = deg2rad(170);
[mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx);

if (size(idx,1)==5)
    disp('5 beacons found - 2nd time')
    %map = [1,0.2,0.2;27,1.06,0.125;29,1.775,0.695];
    distCallLoc = getDistressCall(map,mu,idx);
    disp('moving to distress call')
    [mu,sigma,idx] = toPointDistress(pb, distCallLoc, mu, sigma, idx);

    hold on; plot(distCallLoc(1,1),distCallLoc(1,2),'yx')
    figure;plot_robot(mu,sigma);hold on; plot_beacons(mu,sigma,idx);hold on;plot(distCallLoc(1,1),distCallLoc(1,2),'y*');title('Final Plot')
    realLoc = load('map.mat');
    hold on;plot(realLoc.map(3:end,2),realLoc.map(3:end,3),'x');
    toc;
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

