function ekf_localization()
clear all
close all
load_data()
% this simulator runs for 50 steps
nsteps = 50;
% the map is given
map = get_map();
scatter(map(:,1),map(:,2),200,'k*');
hold on
% pose of the robo at time step 0
x = [0; 0; 0]; 

X = zeros(3,nsteps);
X(:,1) = x;

%% Kalman init

% odometry noise
R = [1 0;0 1];
% sensor noise
Q = [1 0;0 1];
sigma = eye(3);


for k = 1:nsteps
    % This function returns the 
    % distance traveled and heading change 
    %  between k-1 and k.
    [delta_d,delta_theta]  = get_odom(k);
    
    % This function returns
    % the range and bearing to each of 
    % the landmarks in the map at time step k.
    z          = sense(k);
    
    %% Kalman start
    X_ = X(:,k) + [delta_d*cos(X(3,k));delta_d*sin(X(3,k));delta_theta];
    
    Jx = [1 0 -delta_d*sin(X_(3));0 1 delta_d*cos(X_(3));0 0 1];
    Ju = [cos(X_(3)) 0;sin(X_(3)) 0;0 1];
    
    sigma = Jx*sigma*Jx'+Ju*R*Ju';
    
    for ii = 1:size(map,1)
        r = sqrt((map(ii,1)-X_(1)).^2+(map(ii,2)-X_(2)).^2);
        G = [-(map(ii,1)-X_(1))/r -(map(ii,2)-X_(2))/r 0; -(map(ii,2)-X_(2))/(r.^2) -(map(ii,1)-X_(1))/(r.^2) -1];
        K = sigma*G'/(G*sigma*G'+Q);
        H = [r;atan2(map(ii,2)-X_(2),map(ii,1)-X_(1))-X_(3)];
        
        error = (z(ii,:)'-H);
        X_ = X_ + K*error;
        sigma = (eye(3)-K*G)*sigma;
    end
    
    X(:,k+1) = X_;
    plot_cov(X_,sigma,3);
    
    %% Kalman end
    
    % This function returns the true pose of the
    % robot at time step k. 
    % DO NOT use this information in 
    % the localization process. 
    % we use this for the evaluation of your localizer.     
    x_true          = ask_the_oracle(k);    
    scatter (x_true(1),x_true(2),'r');
    scatter (X(1,k+1),X(2,k+1),'bx');
end
% -----------Add your functions below this line and use them in the main loop above---

% --------- Do not change the functions below--

function x = ask_the_oracle(k)
    % x is the pose of the robot at time step k
    global X
    x = X(k,:)';
   
function [delta_d,delta_th] = get_odom(k)
    % returnS odometry at time step k
    global odom   
    delta_d   = odom(k,1);
    delta_th  = odom(k,2);
    
function z = sense(k)
    % get the sensor data at time step k
    global sensor
    global num_z 
    idx = 1 + (k-1) * num_z;
    z = sensor(idx:idx+num_z-1,:);
    
function m= get_map()
    global map
    m = map;
    
        
function load_data()
    global X  
    global odom
    global sensor
    global map
    global num_z
    data    =  load('data.mat');
    X         =  data.xr';
    X         = X(2:end,:);
    odom   = data.odom;
    sensor = data.sensor;
    map     = data.map;
    num_z = length(map);
  
   
