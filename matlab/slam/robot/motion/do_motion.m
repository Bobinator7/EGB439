function [delta_d,delta_theta] = do_motion(pb,vL,vR,time)
    ticks_prev = pb.getMotorTicks();
    pb.setMotorSpeeds(v2pR(vR),v2pL(vL));
    pause(time);
    pb.setMotorSpeeds(0,0);
    ticks_next = pb.getMotorTicks();
    [delta_d,delta_theta] = get_odom_robot(ticks_prev(1),ticks_next(1),ticks_prev(2),ticks_next(2));
end