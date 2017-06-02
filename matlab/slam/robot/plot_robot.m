function plot_robot(mu,sigma)
    mu_ = mu(1:3,1);
    sigma_ = sigma(1:3,1:3);
    
    plot_pose(mu_);
    plot_cov(mu_(1:2),sigma_(1:2,1:2),3,'b');
end