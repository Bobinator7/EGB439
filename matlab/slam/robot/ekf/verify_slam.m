function ekf_slam()
    clear all
    close all
    figure(1)
    grid on; axis equal; 
    axis([-0.5 2.5 -0.5 2.5]);
    hold on

    load_data()
    % this simulator runs for 50 steps
    nsteps = 50;

    % assume motion error with 1 cm standard deviation for translation and 3 degrees
    %for rotation.
    % assume measurement error of 10 cm for range and 3 degreen for bearing  

    global R
    global Q
    
    % the true positions of the landmarks. DO NOT use in the filter
    map = ask_the_oracle_for_map();
    scatter(map(:,1),map(:,2),100,'k*');
    
    %% pose init
    x0 = [0; 0; 0];
    x = zeros(3,nsteps);
    x(:,1) = x0;

    %% Kalman init

    % estimation
    mu = x0;
    sigma = eye(3)*0.01;
    
    %% Beacon init
    idx = [];
    
    for k = 1:nsteps    
        % The true pose of the robot (DO NOT USE in the filter)
         xr          = ask_the_oracle_for_pose(k);    
         scatter(xr(1),xr(2),'r')

        [delta_d,delta_theta]  = get_odom(k);      
        z          = sense(k);
        z = [z,([35 29 54 12 43])'];
        
        [mu,sigma] = prediction_step(mu,sigma,delta_d,delta_theta);
        [mu,sigma,idx] = update_step(mu,sigma,z,idx);
        
        scatter(mu(1,1),mu(2,1),'g+');
        plot_cov(mu(1:2,1),sigma(1:2,1:2),3,'g');
    end
    
    scatter(mu(4,1),mu(5,1),'bx');
    plot_cov(mu(4:5,1),sigma(4:5,4:5),3,'b');
    scatter(mu(6,1),mu(7,1),'bx');
    plot_cov(mu(6:7,1),sigma(6:7,6:7),3,'b');
    scatter(mu(8,1),mu(9,1),'bx');
    plot_cov(mu(8:9,1),sigma(8:9,8:9),3,'b');
    scatter(mu(10,1),mu(11,1),'bx');
    plot_cov(mu(10:11,1),sigma(10:11,10:11),3,'b');
    scatter(mu(12,1),mu(13,1),'bx');
    plot_cov(mu(12:13,1),sigma(12:13,12:13),3,'b');

end
% -----------Add your functions below this line and use them in the main loop above---

 
% --------- Do not change the functions below--

function x = ask_the_oracle_for_pose(k)
    % x is the pose of the robot at time step k
    global X
    x = X(k,:)';
    
end
   
function [delta_d,delta_th] = get_odom(k)
    % returnS odometry at time step k
    global odom   
    delta_d   = odom(k,1);
    delta_th  = odom(k,2);
    
end
    
function z = sense(k)
    % get the sensor data at time step k
    global sensor
    global num_z 
    idx = 1 + (k-1) * num_z;
    z = sensor(idx:idx+num_z-1,:);
    
end
    
function m= ask_the_oracle_for_map()
    global map
    m = map;
    
end
    
        
function load_data()
    global X  
    global odom
    global sensor
    global map
    global num_z
    data    =  load('data2.mat');
    X         =  data.xr';
    X         = X(2:end,:);
    odom   = data.odom;
    sensor = data.sensor;
    map     = data.map;
    num_z = length(map);
end
   
