function img = getImage()
%IP = '172.19.232.163';
IP = '192.168.43.154';
pb = PiBot(IP);
img = flipud(pb.getImageFromCamera());
img = imrotate(img,-90);

end

