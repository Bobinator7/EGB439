function plot_pose(x)
    l = 0.1;
    red_axis_x = linspace(0,l*cos(x(3)),10);
    red_axis_y = linspace(0,l*sin(x(3)),10);
    blue_axis_x = linspace(0,-l*sin(x(3)),10);
    blue_axis_y = linspace(0,l*cos(x(3)),10);
    
    plot(red_axis_x,red_axis_y,'r','LineWidth',2);
    hold on
    plot(blue_axis_x,blue_axis_y,'b','LineWidth',2);
end