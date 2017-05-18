function [mu,sigma] = prediction_step(mu,sigma,delta_d,delta_theta)
    [R,~] = getModelParam();
    
    mu(1:3,1) = mu(1:3,1) + [delta_d*cos(mu(3,1));delta_d*sin(mu(3,1));delta_theta];
    mu(3,1) = wrapToPi(mu(3,1));
    
    Jx_ = [1 0 -delta_d*sin(mu(3,1));0 1 delta_d*cos(mu(3,1));0 0 1];
    Ju_ = [cos(mu(3,1)) 0;sin(mu(3,1)) 0;0 1];
    
    m = size(sigma,1)- size(Jx_,1);
    n = size(sigma,1)- size(Ju_,1);
    
    Jx = [Jx_,zeros(3,m);zeros(m,3),eye(m)];
    Ju = [Ju_;zeros(n,2)];
    
    sigma = Jx*sigma*Jx'+Ju*R*Ju';
end