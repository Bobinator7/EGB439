function call_location = getDistressCall(map,mu,idx)
    beaconid1 = map(2,1);
    idx1 = find(idx==beaconid1);
    idx1 = 2*(idx1-1)+4;
    robot_beacon1 = mu(idx1:idx1+1)';
        
    beaconid2 = map(3,1);
    idx2 = find(idx==beaconid2);
    idx2 = 2*(idx2-1)+4;
    robot_beacon2 = mu(idx2:idx2+1)';
    
    robot_beacon = [robot_beacon1;robot_beacon2];
    guys_beacon = map(2:3,2:3);
    
    tr = getTransform(robot_beacon,guys_beacon);
    
    distress_in_guy_frame = map(1,2:3);
    
    distress_in_robot_frame = distress_in_guy_frame * tr.T + tr.c(1,:);
    
    call_location = distress_in_robot_frame;
end