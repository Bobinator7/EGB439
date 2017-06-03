function [mu,sigma,idx] = rotate(pb, target_angle, mu, sigma, idx)
    %% motion control init
    Kv = 0.7;  
    Kh = 1.0;
    goal_tolerance = 0.3;
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
    
    theta_target = target_angle - mu(3,1);
    dist2target = 0; 
    count =0;
    %% motion loop
    while true
        %% do motion
%         [delta_d,delta_theta] = do_motion(pb,vL,vR,dt);
        [delta_d,delta_theta] = do_motionV2(dist2target, theta_target,pb);
        
        %% get sensor data
        
        img = flipud(pb.getImageFromCamera());
        img = imrotate(img,-90);
        
        z = [];
        count = count + 1;
        if count >= 3
            z = senseless(img);
            count =0;
        end
        
        %% Kalman Filter
        [mu,sigma] = prediction_step(mu,sigma,delta_d,delta_theta);
        plot_robot(mu,sigma);
        plot_beacons(mu,sigma,idx);  
        [mu,sigma,idx] = update_step(mu,sigma,z,idx);
        plot_robot(mu,sigma);
        plot_beacons(mu,sigma,idx);  
                
        %% break when goal is reached & calculate next motion parameters
    
        theta_target = target_angle - mu(3,1);
        if (abs(theta_target) < goal_tolerance)
            break
        end
        
%         velAv = Kv * dist2target;
%         velDiff = Kh * (theta_target - mu(3,1));
%         vL = velAv - velDiff/2;
%         vR = velAv + velDiff/2;
%         if vL > 0.5
%             vL = 0.5;
%         end
%         if vR > 0.5
%             vR = 0.5;
%         end
%         if vL < -0.5
%             vL = -0.5;
%         end
%         if vR < -0.5
%             vR = -0.5;
%         end
        
        %% plot stuff
        %plot_robot(mu,sigma);
        %plot_beacons(mu,sigma,idx);  

    end
    
  
    
    
    %% stop motors after motion
    pb.setMotorSpeeds(0,0);

end