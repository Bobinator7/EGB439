%% Image Processing - EGB439
clear all;clc;close all;

%% import image
img = imread('C:\Users\Callum\Documents\University\EGB439\EGB439\matlab\images\img_2.jpg');

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
figure(1);imshow(chrom(:,:,3)<0.8);
red = imopen(chrom(:,:,1)<0.8,se);
green = imopen(chrom(:,:,2)<0.8,se);
blue = imopen(chrom(:,:,3)<0.8,se);
blue = imopen(blue,seB);
blue = imopen(blue,seB2);
figure(2);imshow(blue);
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
global it;
for i=1:size(allRedBlobAreas,1)
    if (green(round(allRedBlobAreas(i).Centroid(2)-allRedBlobAreas(i).MinorAxisLength), round(allRedBlobAreas(i).Centroid(1))))
        greenBlobs = cat(1,greenBlobs,[round(allRedBlobAreas(i).Centroid(1)), round(allRedBlobAreas(i).Centroid(2))-allRedBlobAreas(i).MinorAxisLength]);
    end
    if (green(round(allRedBlobAreas(i).Centroid(2)+allRedBlobAreas(i).MinorAxisLength), round(allRedBlobAreas(i).Centroid(1))))
        greenBlobs = cat(1,greenBlobs,[round(allRedBlobAreas(i).Centroid(1)), round(allRedBlobAreas(i).Centroid(2)+allRedBlobAreas(i).MinorAxisLength)]);
    end
    it = i;
end
for i=i:size(allBlueBlobAreas,1)
    if (green(round(allBlueBlobAreas(i).Centroid(2)-allBlueBlobAreas(i).MinorAxisLength), round(allBlueBlobAreas(i).Centroid(1))))
        greenBlobs = cat(1,greenBlobs,[round(allBlueBlobAreas(i).Centroid(1)), round(allBlueBlobAreas(i).Centroid(2)-allBlueBlobAreas(i).MinorAxisLength)]);
    end
    if (green(round(allBlueBlobAreas(i).Centroid(2)+allBlueBlobAreas(i).MinorAxisLength), round(allBlueBlobAreas(i).Centroid(1))))
        greenBlobs = cat(1,greenBlobs,[round(allBlueBlobAreas(i).Centroid(1)), round(allBlueBlobAreas(i).Centroid(2)+allBlueBlobAreas(i).MinorAxisLength)]);
    end
end
% NOTE: both blue and red will find the green spot
% TODO: add filter to  remove points from greenBlobs if within a close
% proximity to another point, i.e., within 5 pixels 
% TODO: pick up small blue blobs

%% show images

%% Keypoints of beacons
% kp_list = [];
% for it = 1:numel(allRedBlobAreas)
%     kp_list = [kp_list;allRedBlobAreas(it).Centroid, 1, allRedBlobAreas(it).MajorAxisLength];
% end
% for it = 1:numel(allGreenBlobAreas)
%     kp_list = [kp_list;allGreenBlobAreas(it).Centroid, 2, allGreenBlobAreas(it).MajorAxisLength];
% end
% for it = 1:numel(allBlueBlobAreas)
%     kp_list = [kp_list;allBlueBlobAreas(it).Centroid, 3, allBlueBlobAreas(it).MajorAxisLength];
% end
% 
% %% sort rows per pixel location of 1st column
% kp_list = sortrows(kp_list,[1]);
% 
% 
% %result = chunks()
% beacon1 = kp_list(1:3);
% 
% disp(kp_list)

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
    

