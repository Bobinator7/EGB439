%% Image Processing - EGB439
function z = senseless(img)
%clear all;clc;close all;

%% import image
%img = imread('C:\Users\Callum\Documents\University\EGB439\EGB439\matlab\images\img_7.jpg');

%% blur image
imgF = imgaussfilt(img, 1);

%% Gamma correct the image
imgGamma = gamma_correction(imgF, 10);

%% Calculate the chromoticity
[chrom, lum] = getChrom(imgGamma);

%% Threshold
%red = im2bw(chrom(:,:,1),0.6);
% blue = im2bw(chrom(:,:,3),0.6);

%% Dilate blobs
se = strel('diamond',2);
seB = strel('line',4,90);
seB2 = strel('line',4,0);
red = imopen(chrom(:,:,1)<0.8,se);
green = imopen(chrom(:,:,2)<0.8,se);
blue = imopen(chrom(:,:,3)<0.8,se);
blue = imopen(blue,seB);
blue = imopen(blue,seB2);
blue = 1-blue;
red=1-red;

%% Erode image
% se = strel('disk',1);
% red = imerode(red,se);
% blue = imerode(blue,se);

%% Remove any blobs smaller than ...
%red = bwareafilt(redBW,[50,1000]);
%blue = bwareafilt(blueBW,[50,1000]);

%% Identify red,green,blue blobs
[oneR, twoR] = bwlabel(red, 4);
allRedBlobAreas = regionprops(oneR,'all');
[oneB, twoB] = bwlabel(blue, 4);
allBlueBlobAreas = regionprops(oneB,'all');

%% Green Keypoints
% red loop for find green keypoints
green = 1 - green; % make green blob 1 and other 0
greenBlobs = [];
for i=1:size(allRedBlobAreas,1)
    if (green(round(allRedBlobAreas(i).Centroid(2)-allRedBlobAreas(i).MinorAxisLength), round(allRedBlobAreas(i).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allRedBlobAreas(i).Centroid(1)), round(allRedBlobAreas(i).Centroid(2))-allRedBlobAreas(i).MinorAxisLength]));
    end
    if (green(round(allRedBlobAreas(i).Centroid(2)+allRedBlobAreas(i).MinorAxisLength), round(allRedBlobAreas(i).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allRedBlobAreas(i).Centroid(1)), round(allRedBlobAreas(i).Centroid(2)+allRedBlobAreas(i).MinorAxisLength)]));
    end
    it = i;
end
for i=i:size(allBlueBlobAreas,1)
    if (green(round(allBlueBlobAreas(i).Centroid(2)-allBlueBlobAreas(i).MinorAxisLength), round(allBlueBlobAreas(i).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allBlueBlobAreas(i).Centroid(1)), round(allBlueBlobAreas(i).Centroid(2)-allBlueBlobAreas(i).MinorAxisLength)]));
    end
    if (green(round(allBlueBlobAreas(i).Centroid(2)+allBlueBlobAreas(i).MinorAxisLength), round(allBlueBlobAreas(i).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allBlueBlobAreas(i).Centroid(1)), round(allBlueBlobAreas(i).Centroid(2)+allBlueBlobAreas(i).MinorAxisLength)]));
    end
end
% One green point if in proximity.
if (size(greenBlobs,1)>1)
    for i=1:(size(greenBlobs,1)-1)
        if (abs(greenBlobs(i,1)-greenBlobs(i+1,1))<5)&&(abs(greenBlobs(i,2)-greenBlobs(i+1,2))<5)
            greenBlobs(i+1,:)=[];
        end
    end
end
% TODO: pick up small blue blobs - tempramental
% TODO: pick up small green

%% show images
figure;imshow(chrom);
for j=1:size(greenBlobs,1)
    hold on; plot(greenBlobs(i,1),greenBlobs(i,2),'o--k')
end
%% Keypoints of beacons
kp_list = [];
for it = 1:numel(allRedBlobAreas)
    kp_list = round([kp_list;allRedBlobAreas(it).Centroid, 1, allRedBlobAreas(it).MajorAxisLength]);
end
for it = 1:size(greenBlobs,1)
    kp_list = round([kp_list;[greenBlobs(it,1) greenBlobs(it,2)], 2, 0]); %Centroid for green not known - default 0.
end
for it = 1:numel(allBlueBlobAreas)
    kp_list = round([kp_list;allBlueBlobAreas(it).Centroid, 3, allBlueBlobAreas(it).MajorAxisLength]);
end

%% sort rows per pixel location of 1st column
kp_list = sortrows(kp_list,[1]);

%% Remove not complete beacon from kp_list
tolerance = 5;
m1 = -0.0210896309314587;
b1 = 1.5588466973637962;
m2 = -0.142649154;
b2 = 1.530568848;
x_current = 0;listA=[];listB=[];z=[];
for ii=1:size(kp_list,1)
    if abs(x_current - kp_list(ii,1))>tolerance
        listA = [];
        x_current = kp_list(ii,1);
        listA = cat(1,listA,kp_list(ii,:));
        continue
    else
        listA = cat(1,listA,kp_list(ii,:));
        if size(listA,1)>=3
            listA = sortrows(listA,[2]);
            ID = bitshift(listA(1,3),4)+bitshift(listA(2,3),2)+bitshift(listA(3,3),0);
            meanX = sum(listA(:,1))/3;
            meanY = sum(listA(:,2))/3;
            distY = listA(3,2)-listA(1,2);
            ran = m1*(distY)+b1;
            bearing = m2*((meanX-160)/ran)+b2;
            beacon = [ran, bearing, ID];
            z = cat(1,z,beacon);
            listB = cat(1,listB,listA);
            listA = [];
        end
    end
end
end
