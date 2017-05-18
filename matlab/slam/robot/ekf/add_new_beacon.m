function [mu,sigma,idx] = add_new_beacon(mu,sigma,z,idx)
    [~,Q] = getModelParam();

    mu1 = mu(1,1) + z(1,1)*cos(mu(3,1)+z(1,2));
    mu2 = mu(2,1) + z(1,1)*sin(mu(3,1)+z(1,2));
    mu = [mu;mu1;mu2];
    idx = [idx;z(1,3)];
    L = [cos(mu(3,1)+z(1,2)),-z(1,1)*sin(mu(3,1)+z(1,2)) ;sin(mu(3,1)+z(1,2)) ,z(1,1)*cos(mu(3,1)+z(1,2))];
    sigma_ = L*Q*L';
    sigma = [sigma,zeros(size(sigma,1),2);zeros(2,size(sigma,2)),sigma_];
end