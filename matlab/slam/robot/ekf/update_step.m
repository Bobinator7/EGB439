function [mu,sigma,idx] = update_step(mu,sigma,z,idx)
    [~,Q] = getModelParam();
   
    %for ii = 0:(size(z,1)-1)
    for ii = 1:size(z,1)
        % TODO: change condition when to add a beaon DONE!
        % TODO: change way to index mu with respective beacon DONE!
        % TODO: change index ii to 1:size(z,1) DONE!
        %z = [r,b,idx;...];
        %~size(find(idx==z(ii,3)),1)
        
        %if ii*2+2+3 > size(mu,1)
        mu_idx = find(idx==z(ii,3));
        mu_idx1 = 2*mu_idx-1 + 3;
        mu_idx2 = 2*mu_idx + 3;
        if ~size(mu_idx,1)
            
            %[mu, sigma] = add_new_beacon(mu,sigma,z,ii);
            [mu, sigma,idx] = add_new_beacon(mu,sigma,z(ii,:),idx);
            
            %scatter(mu(ii*2+1+3,1),mu(ii*2+2+3,1),'bx');
            %plot_cov(mu(ii*2+1+3:ii*2+2+3,1),sigma(ii*2+1+3:ii*2+2+3,ii*2+1+3:ii*2+2+3),3);
            mu_idx = find(idx==z(ii,3));
            mu_idx1 = 2*mu_idx-1 + 3;
            mu_idx2 = 2*mu_idx + 3;
            scatter(mu(mu_idx1,1),mu(mu_idx2,1),'bx');
            plot_cov(mu(mu_idx1:mu_idx2,1),sigma(mu_idx1:mu_idx2,mu_idx1:mu_idx2),3,'b');
        else
            % calculate G
            xr = mu(1:2,1);
            %xb = mu(2*ii+1+3:2*ii+2+3,1);
            xb = mu(mu_idx1:mu_idx2,1);
            
            r = sqrt((xb(1)-xr(1)).^2+(xb(2)-xr(2)).^2);
            G1 = [-(xb(1)-xr(1))/r -(xb(2)-xr(2))/r 0;
                  (xb(2)-xr(2))/(r.^2) -(xb(1)-xr(1))/(r.^2) -1];
            G2(1,1) = (xb(1)-xr(1))/r;
            G2(2,1) = -(xb(2)-xr(2))/(r.^2);
            G2(1,2) = (xb(2)-xr(2))/r;
            G2(2,2) = (xb(1)-xr(1))/(r.^2);
            G = zeros(2,size(sigma,1));
            G(1:2,1:3) = G1;
            %G(1:2,2*ii+1+3:2*ii+2+3) = G2;
            G(1:2,mu_idx1:mu_idx2) = G2;
            
            % calculate K
            K = sigma*G'*inv(G*sigma*G'+Q);
            
            % calculate h
            %h = calc_h(mu(2*ii+1+3:2*ii+2+3),mu(1:3));
            h = calc_h(mu(mu_idx1:mu_idx2),mu(1:3));
            
            % predict!
            error = z(ii,1:2)'-h;
            error(2) = wrapToPi(error(2));
            mu = mu + K*error;
            mu(3,1) = wrapToPi(mu(3,1));
            sigma = (eye(size(sigma))-K*G)*sigma;
        end
    end
    
end