power = 0:-10:-100;

%% plain floor data
vl = [0 0 0.041 0.108 0.196 .289 .378 .470 .562 .641 .730];
vr = [0 0 .040 .107 .195 .283 .368 .463 .548 .632 .713];
%figure;
%plot(power,vl,'b+',power,vr,'rx');

%% regressed linear fcn (highest order coeff to lowest)
lin_l = polyfit(power(3:end),vl(3:end),1);
lin_r = polyfit(power(3:end),vr(3:end),1);
%figure;
%


