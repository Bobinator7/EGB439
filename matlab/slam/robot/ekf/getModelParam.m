function [R,Q] = getModelParam()
    R = [0.01^2 0;0 deg2rad(1)^2];
    Q = [0.4^2 0;0 deg2rad(5)^2];
end