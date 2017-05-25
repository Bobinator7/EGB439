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

    % odometry noise
    R = [0.01^2 0;0 deg2rad(3)^2];
    % sensor noise
    Q = [0.1^2 0;0 deg2rad(3)^2];
    % estimation
    mu = x0;
    sigma = eye(3)*0.01;
    
    for k = 1:nsteps    
        % The true pose of the robot (DO NOT USE in the filter)
         xr          = ask_the_oracle_for_pose(k);    
         scatter(xr(1),xr(2),'r')

        [delta_d,delta_theta]  = get_odom(k);      
        z          = sense(k);
        
        [mu,sigma] = prediction_step(mu,sigma,delta_d,delta_theta);
        [mu,sigma] = update_step(mu,sigma,z);
        
        scatter(mu(1,1),mu(2,1),'g+');
        plot_cov(mu(1:2,1),sigma(1:2,1:2),1);
    end
    
    scatter(mu(4,1),mu(5,1),'bx');
    plot_cov(mu(4:5,1),sigma(4:5,4:5),3);
    scatter(mu(6,1),mu(7,1),'bx');
    plot_cov(mu(6:7,1),sigma(6:7,6:7),3);
    scatter(mu(8,1),mu(9,1),'bx');
    plot_cov(mu(8:9,1),sigma(8:9,8:9),3);
    scatter(mu(10,1),mu(11,1),'bx');
    plot_cov(mu(10:11,1),sigma(10:11,10:11),3);
    scatter(mu(12,1),mu(13,1),'bx');
    plot_cov(mu(12:13,1),sigma(12:13,12:13),3);

end
% -----------Add your functions below this line and use them in the main loop above---

function [mu,sigma] = add_new_beacon(mu,sigma,z,ii)
    global Q

    mu(2*ii+1+3,1) = mu(1,1) + z(ii+1,1)*cos(mu(3,1)+z(ii+1,2));
    mu(2*ii+2+3,1) = mu(2,1) + z(ii+1,1)*sin(mu(3,1)+z(ii+1,2));
    L = [cos(mu(3,1)+z(ii+1,2)),-z(ii+1,1)*sin(mu(3,1)+z(ii+1,2)) ;sin(mu(3,1)+z(ii+1,2)) ,z(ii+1,1)*cos(mu(3,1)+z(ii+1,2))];
    sigma_ = L*Q*L';
    sigma = [sigma,zeros(size(sigma,1),2);zeros(2,size(sigma,2)),sigma_];
end

function h = predict_map(xb,xr)
    h = zeros(2,1);
    h(1) = sqrt((xb(1)-xr(1))^2+(xb(2)-xr(2))^2);
    h(2) = atan2(xb(2)-xr(2),xb(1)-xr(1))-xr(3);
    h(2) = wrapToPi(h(2));
end

function [mu,sigma] = prediction_step(mu,sigma,delta_d,delta_theta)
    global R
    
    mu(1:3,1) = mu(1:3,1) + [delta_d*cos(mu(3,1));delta_d*sin(mu(3,1));delta_theta];
    mu(3,1) = wrapToPi(mu(3,1));
    
    Jx_ = [1 0 -delta_d*sin(mu(3,1));0 1 delta_d*cos(mu(3,1));0 0 1];
    Ju_ = [cos(mu(3,1)) 0;sin(mu(3,1)) 0;0 1];
    
    m = size(sigma,1)- size(Jx_,1);
    n = size(sigma,1)- size(Ju_,1);
    
    Jx = [Jx_,zeros(3,m);zeros(m,3),eye(m)];
    Ju = [Ju_;zeros(n,2)];
    
    sigma = Jx*sigma*Jx'+Ju*R*Ju';
end

function [mu,sigma] = update_step(mu,sigma,z)
    global Q
   
    for ii = 0:(size(z,1)-1)
        if ii*2+2+3 > size(mu,1)
            [mu, sigma] = add_new_beacon(mu,sigma,z,ii);
            scatter(mu(ii*2+1+3,1),mu(ii*2+2+3,1),'bx');
            plot_cov(mu(ii*2+1+3:ii*2+2+3,1),sigma(ii*2+1+3:ii*2+2+3,ii*2+1+3:ii*2+2+3),3);
        else
            % calculate G
            xr = mu(1:2,1);
            xb = mu(2*ii+1+3:2*ii+2+3,1);
            
            r = sqrt((xb(1)-xr(1)).^2+(xb(2)-xr(2)).^2);
            G1 = [-(xb(1)-xr(1))/r -(xb(2)-xr(2))/r 0;
                  (xb(2)-xr(2))/(r.^2) -(xb(1)-xr(1))/(r.^2) -1];
            G2(1,1) = (xb(1)-xr(1))/r;
            G2(2,1) = -(xb(2)-xr(2))/(r.^2);
            G2(1,2) = (xb(2)-xr(2))/r;
            G2(2,2) = (xb(1)-xr(1))/(r.^2);
            G = zeros(2,size(sigma,1));
            G(1:2,1:3) = G1;
            G(1:2,2*ii+1+3:2*ii+2+3) = G2;
            % calculate K
            K = sigma*G'*inv(G*sigma*G'+Q);
            % calculate h
            h = predict_map(mu(2*ii+1+3:2*ii+2+3),mu(1:3));
            
            % predict!
            error = z(ii+1,:)'-h;
            error(2) = wrapToPi(error(2));
            mu = mu + K*error;
            mu(3,1) = wrapToPi(mu(3,1));
            sigma = (eye(size(sigma))-K*G)*sigma;
        end
    end
    
end

 
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
   
