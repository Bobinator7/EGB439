function [mask] = getGrayMask(img)
    mask = zeros(size(img,1),size(img,2));
    
    for i = 1:size(img,1)
        for j = 1:size(img,2)
        minimum = 255;
        maximum = 0;
        for val = 1:4
            if img(i,j,val)<minimum
                minimum = img(i,j,val);
            end
            if img(i,j,val)> maximum
                maximum = img(i,j,val);
            end
            
        end
        if (maximum - minimum <0.05)
            mask(i,j) = 255;
        end
        end
    end     
end