function pR = v2pR(v)
    if v > 0
        pR = int32(round(v * (-116.6512) - 16.65));
    elseif v < 0
        pR = int32(round(v * (-116.6512) + 16.65));
    else
        pR = 0;
    end
end