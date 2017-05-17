function [chrom, lum] = getChrom(img)
    chrom = zeros(size(img,1), size(img,2));
    lum = zeros(size(img,1),size(img,2));
    
    for i = 1:size(img,1)
        for j = 1:size(img,2)
            b = img(i,j,3);
            g = img(i,j,2);
            r = img(i,j,1);
            sum = r+g+b;
            chrom(i,j,1) = img(i,j,1)/sum;
            chrom(i,j,2) = img(i,j,2)/sum;
            chrom(i,j,3) = img(i,j,3)/sum;
            lum(i,j) = sum; 
        end
    end
end