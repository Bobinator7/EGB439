function result = gamma_connection(img, correction)
im = double(img)/255.0;
result = (im.^correction);
%result = uint8(im);
end