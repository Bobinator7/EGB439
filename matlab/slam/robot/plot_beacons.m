function plot_beacons(mu,sigma,id)

    if size(mu,1) < 4
        return
    end

    mu_ = mu(4:end,1);

    number_of_beacons = size(mu_,1)/2;
    for ii = 0:number_of_beacons-1
        idx = 2*ii+4;
        %scatter(mu(idx,1),mu(idx+1,1),'*');
        text(mu(idx,1),mu(idx+1,1),num2str(id(ii+1)));
        hold on;
        plot_cov(mu(idx:idx+1,1),sigma(idx:idx+1,idx:idx+1),3,'g');
    end
end