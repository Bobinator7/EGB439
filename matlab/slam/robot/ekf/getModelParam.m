function [R,Q] = getModelParam()
    R = [0.01^2 0;0 deg2rad(1)^2]; % enco
    Q = [0.2^2 0;0 deg2rad(5)^2]; % camera
end