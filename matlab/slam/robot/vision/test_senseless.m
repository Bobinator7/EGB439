
%pb = startup_robot('192.168.43.154');

%img = getImage;
%cd ../../..
%img = imread('beaconImage250.jpg');
imshow(img);
%img = pb.getImageFromCamera();
%img = imrotate(img,-90);

z = senseless(img)