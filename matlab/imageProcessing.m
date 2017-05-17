%% Image Processing
clear all;clc;close all;
img = imread('beacons.jpg');
img = imgaussfilt(img, 1);

%greenFilter = img;
greenFilter = img(:,:,1)>55 & img(:,:,2)>70 & img(:,:,3)<29;
gF = strel('line',2,10);
gF2= strel('line',10,2);
test = medfilt2(greenFilter, [6 6]);

greenFilterDilate = imdilate(greenFilter,[gF,gF2]);
greenFilterDilate = 1 - greenFilterDilate;
greenFloor = img(:,:,1)<40 & img(:,:,2)>75 & img(:,:,3)<42;

sD1 = strel('line',100,5);
sD2 = strel('line',5,70);
greenFloorDilate = imdilate(greenFloor,[sD1,sD2]);
figure(1)
imshow(img)
imgGamma = gamma_connection(img, 10);
figure(2)
imshow(imgGamma)
[chrom, lum] = getChrom(imgGamma);

se = strel('disk',3);
red = imopen(chrom(:,:,1)<0.8,se);
green = imopen(chrom(:,:,2)<0.8, se);
blue = imopen(chrom(:,:,3)<0.8,se);


[oneR, twoR] = bwlabel(1-red, 8);
allRedBlobAreas = regionprops(oneR,'all');
[oneG, twoG] = bwlabel(test, 8);
allGreenBlobAreas = regionprops(oneG,'all');
[oneB, twoB] = bwlabel(1-blue, 8);
allBlueBlobAreas = regionprops(oneB,'all');


new = red & green & blue;


% figure(3)
% imshow(red)
% figure(4)
% imshow(green)
% figure(5)
% imshow(blue)
% figure(6)
% imshow(new)
%m_r = getBeaconID(img); 

figure(7)
imshow(greenFilter)
figure(8)
imshow(greenFilterDilate)
figure(9)
imshow(greenFloor)
figure(10)
imshow(greenFloorDilate)
figure(11)
imshow(test)

kp_list = [];
for it = 1:numel(allRedBlobAreas)
    kp_list = [kp_list;allRedBlobAreas(it).Centroid, 1, allRedBlobAreas(it).MajorAxisLength];
end
for it = 1:numel(allGreenBlobAreas)
    kp_list = [kp_list;allGreenBlobAreas(it).Centroid, 2, allGreenBlobAreas(it).MajorAxisLength];
end
for it = 1:numel(allBlueBlobAreas)
    kp_list = [kp_list;allBlueBlobAreas(it).Centroid, 3, allBlueBlobAreas(it).MajorAxisLength];
end

%kp_list.sort()
kp_list = sortrows(kp_list,[1]);
beacon1 = kp_list(1:3)

disp(kp_list)

% %% TODO: Add chunks rejector
% for it in kp_list:
%     ID = 0
%     meanX = 0
%     bigY = 0
%     smallY = 900
%     for m = 1:3
%         % get ID
%         ID = ID + it[m][2] * 2**((2-m)*2)
% 
%         % get horizontal position
%         meanX = (meanX + it[m][0])
% 
%         % get size of beacon
%         if smallY > it[m][1]:
%             smallY = it[m][1]
%         if bigY < it[m][1]:
%             bigY = it[m][1]
% 
%     
%     meanX = meanX / 3
% 
%     m1 = -0.0210896309314587
%     b1 = 1.5588466973637962
%     ran = m1*(bigY-smallY)+b1
%     m2 = -0.142649154
%     b2 = 1.530568848
%     bearing = m2*((meanX-160)/r)+b2
%     result = np.append(result,[[ID, ran, bearing ]],axis=0)
%     count = count + 1
% 
% return result
    

