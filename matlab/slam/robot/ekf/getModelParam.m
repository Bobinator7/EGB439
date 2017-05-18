function [R,Q] = getModelParam()
    R = [0.01^2 0;0 deg2rad(3)^2];
    Q = [0.1^2 0;0 deg2rad(3)^2];
end