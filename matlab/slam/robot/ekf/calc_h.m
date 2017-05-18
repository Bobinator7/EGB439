function h = predict_map(xb,xr)
    h = zeros(2,1);
    h(1) = sqrt((xb(1)-xr(1))^2+(xb(2)-xr(2))^2);
    h(2) = atan2(xb(2)-xr(2),xb(1)-xr(1))-xr(3);
    h(2) = wrapToPi(h(2));
end