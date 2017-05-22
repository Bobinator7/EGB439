%% Test 

% robot init
startup_robot('172.19.232.163');

% EKF init
idx = [];
mu = [0;0;0];
sigma = eye(3);

% 
target = [1;1];
[mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx);