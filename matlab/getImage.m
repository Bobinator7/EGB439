function img = getImage()
IP = '172.19.232.105';
pb = PiBot(IP);
img = imrotate(pb.getImageFromCamera(),-90);
end

