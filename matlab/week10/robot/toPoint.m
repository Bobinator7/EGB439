function end_configuration = toPoint(pb, target, start_configuration)
    %% motion control init
    Kv = 0.7;  
    Kh = 1.0;
    goal_tolerance = 0.1;
    dt = 0.2;
    vL = 0;
    vR = 0;
    
    %% Kalman init
    % odometry noise
    R = [1 0;0 1];
    % sensor noise
    Q = [1 0;0 1];
    sigma = eye(3);
    % start configuration
    X = start_configuration(:);
    
    %% motion loop
    while true
        %% do motion
        [delta_d,delta_theta] = do_motion(pb,vL,vR,dt);
        
        %% get sensor data
        
        %% Kalman Filter
        X_ = X + [delta_d*cos(X(3));delta_d*sin(X(3));delta_theta];
    
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
        
        X = X_;
        
        %% break when goal is reached
        dist2target = sqrt((target(1)-X(1))^2+(target(2)-X(2))^2);
        if (dist2target < goal_tolerance)
            break
        end
                  
        %% calculate next motion parameters
        theta_target = atan2((target(2)-X(2)),(target(1)-X(1)));
        velAv = Kv * dist2target;
        velDiff = Kh * (theta_target - X(3));
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

    end
    
    %% stop motors after motion
    pb.setMotorSpeeds(0,0);
    
    end_configuration = X_;
end