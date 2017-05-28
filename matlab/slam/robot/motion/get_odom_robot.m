function [delta_d,delta_theta] = get_odom_robot(tRp,tRn,tLp,tLn)
    wheel_dia = 0.065;
    turn_dia = 0.12;%0.145

    diffL = enc_diff(tLp,tLn);
    diffR = enc_diff(tRp,tRn);
    
    delta_d = wheel_dia*pi*(diffL + diffR)/(2*360);
    delta_theta = ((diffR - diffL)/180)*(wheel_dia/turn_dia);
end