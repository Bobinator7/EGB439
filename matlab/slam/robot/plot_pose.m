function plot_pose(x)
    l = 0.1;
    red_axis_x = linspace(x(1),x(1)+l*cos(x(3)),10);
    red_axis_y = linspace(x(2),x(2)+l*sin(x(3)),10);
  
    %[theta, r] = cart2pol(red_axis_x, red_axis_y);
    
    %theta = theta + pi/2;
    
    %[blue_axis_x, blue_axis_y] = pol2cart(theta, r);
    
    blue_axis_x = linspace(x(1),x(1)-l*sin(x(3)),10);
    blue_axis_y = linspace(x(2),x(2)+l*cos(x(3)),10);
      
    plot(red_axis_x,red_axis_y,'r','LineWidth',2);
    hold on
    plot(blue_axis_x,blue_axis_y,'b','LineWidth',2);
end