function [mu,sigma,idx] = toPoint(pb, target, mu, sigma, idx)
    %% motion control init
    Kv = 0.7;  
    Kh = 1.0;
    goal_tolerance = 0.1;
    dt = 0.2;
    vL = 0;
    vR = 0;
    
    %% Kalman init
    % odometry noise
    %R = [1 0;0 1];
    % sensor noise
    %Q = [1 0;0 1];
    %sigma = eye(3);
    % start configuration
    %X = start_configuration(:);
    %idx = []
    
    %% motion loop
    while true
        %% do motion
        [delta_d,delta_theta] = do_motion(pb,vL,vR,dt);

        
        %% get sensor data
        
        img = pb.getImageFromCamera();
        img = imrotate(img,-90);
        z = senseless(img);
        
        %% Kalman Filter
        [mu,sigma] = prediction_step(mu,sigma,delta_d,delta_theta);
        [mu,sigma,idx] = update_step(mu,sigma,z,idx);
                
        %% break when goal is reached
        dist2target = sqrt((target(1)-mu(1,1))^2+(target(2)-mu(2,1))^2);
        if (dist2target < goal_tolerance)
            break
        end
                  
        %% calculate next motion parameters
        theta_target = atan2((target(2)-mu(2,1)),(target(1)-mu(1,1)));
        velAv = Kv * dist2target;
        velDiff = Kh * (theta_target - mu(3,1));
        vL = velAv - velDiff/2;
        vR = velAv + velDiff/2;
        if vL > 0.5
            vL = 0.5;
        end
        if vR > 0.5
            vR = 0.5;
        end
        if vL < -0.5
            vL = -0.5;
        end
        if vR < -0.5
            vR = -0.5;
        end
        
        %% plot stuff
        plot_robot(mu,sigma);
        plot_beacons(mu,sigma,idx);  

    end
    
  
    
    
    %% stop motors after motion
    pb.setMotorSpeeds(0,0);

end