function pL = v2pL(v)
    if v > 0
        pL = int32(round(v * (-122.00) - 16.75));
    elseif v < 0
        pL = int32(round(v * (-122.00) + 16.75));
    else
        pL = 0;
    end
end