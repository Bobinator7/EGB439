function ekf_localization_robot()
clear all
close all

% extract data
load('map.mat');
x = [map(1,2:3)'; 0]; 
goals = map(2:3,2:3);
beacons = map(4:end,1:3);
scatter(beacons(:,2),beacons(:,3),200,'k*');
hold on
scatter(goals(:,1),goals(:,2),200,'rx');
plot_pose(x);

%x = toPoint(pb,)

end

function plot_pose(x)
    l = 0.07;
    red_axis_x = x(1) + linspace(0,l*cos(x(3)),10);
    red_axis_y = x(2) + linspace(0,l*sin(x(3)),10);
    blue_axis_x = x(1) + linspace(0,-l*sin(x(3)),10);
    blue_axis_y = x(2) + linspace(0,l*cos(x(3)),10);
    
    plot(red_axis_x,red_axis_y,'r','LineWidth',2);
    plot(blue_axis_x,blue_axis_y,'b','LineWidth',2);
end
