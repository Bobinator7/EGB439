function tr = getTransform(robot_beacons,guy_beacons)
% beacons are in the following formatting:
% robot_beacons = [point1;point2;...]
% guy_beacons = [point1;point2;...]
% point1 = [x y] is the position of the same beacon in their respective frame

if size(robot_beacons,2) ~= 2 && size(robot_beacons,1) < 2
    disp('Number of beacons smaller 2 or dimension of point is not 2! Abort..');
    return
end

[~,~,tr] = procrustes(robot_beacons,guy_beacons,'scaling',false,'reflection',false);

   

end