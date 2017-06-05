function [R,Q] = getModelParam()
    R = [0.01^2 0;0 deg2rad(2.0)^2]; % enco
    Q = [0.3^2 0;0 deg2rad(10)^2]; % camera
end