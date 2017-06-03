function [delta_d,delta_theta] = do_motionV2(dist2target, theta_target, pb)
    ang_tol = 0.2;
    dist_tol = 0.1; %0.1 
    time = 0.3; %0.2
    acc_time = 0.1;
    ticks_prev = pb.getMotorTicks();
    p = 40;
     if theta_target > 0 
        l = -p; r = p;
     else
        l = p; r = -p;
    end
    if abs(theta_target) > ang_tol
        pb.setMotorSpeeds([l r],time,acc_time)
    else
        if dist2target > dist_tol
            pb.setMotorSpeeds([-50 -50],time,acc_time);
        end
    end
    ticks_next = pb.getMotorTicks();
    [delta_d,delta_theta] = get_odom_robot(ticks_prev(1),ticks_next(1),ticks_prev(2),ticks_next(2));
    
%     ticks_prev = pb.getMotorTicks();
%     pb.setMotorSpeeds([v2pR(vR),v2pL(vL)],time);
%     %pb.setMotorSpeeds(0,0)
%     ticks_next = pb.getMotorTicks();
%     [delta_d,delta_theta] = get_odom_robot(ticks_prev(1),ticks_next(1),ticks_prev(2),ticks_next(2));
end