function res = enc_diff(count_prev,count_next)
    half_res = 15360;
    full_res = 30720;
    scaled_count = count_next - count_prev + half_res;
    res = count_next - count_prev;
    
    if scaled_count < 0
        res = (half_res - count_prev) + (count_next + half_res);
    elseif scaled_count >= full_res
        res = (half_res - count_next) + (count_prev + half_res);
    end
end