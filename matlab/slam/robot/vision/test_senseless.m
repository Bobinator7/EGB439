
%pb = startup_robot('172.19.232.105');

img = getImage;
%cd ../../..
%img = imread('beaconImage250.jpg');
imshow(img);
%img = pb.getImageFromCamera();
%img = imrotate(img,-90);

z = senseless(img)