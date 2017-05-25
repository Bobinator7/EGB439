function chunk = chunks(l,n)
    %Yield successive n-sized chunks from l.%
    tolerance = 3;
    chunk = []
    
    i=0;
    while (i<length(l))
        current_x = 0;
        for j =0:n
            if i+j >= length(l)
                break
            end
        end
        if j == 0 
            current_x = l(i+j,1);
        else
            if l(i+j,1) - current_x > tolerance
                break
            end
        end
        if i+j >= len(l)
            break
        end
        
        if l(i+j,1) - current_x > tolerance
            i = i+j;
            continue
        else
            chunk = [chunk;l(i:i+n)];
        end
    end
end