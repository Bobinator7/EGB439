%% Image Processing - EGB439
function z = senseless(img)
if isempty(img)
    z =[];
    return 
end

%% real beacons
idReal = [27 29 38 45 57];

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
kdick = 1.02;
for ii=1:size(allRedBlobAreas,1)
    idx1 = subplus(round(allRedBlobAreas(ii).Centroid(2)-allRedBlobAreas(ii).MinorAxisLength*kdick)) +1;
    idx2 = subplus(round(allRedBlobAreas(ii).Centroid(2)+allRedBlobAreas(ii).MinorAxisLength*kdick)) +1;
    if idx2 > size(green,1)
        idx2 = size(green,1);
    end
    if (green(idx1, round(allRedBlobAreas(ii).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allRedBlobAreas(ii).Centroid(1)), idx1]));
    end
    if (green(idx2, round(allRedBlobAreas(ii).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allRedBlobAreas(ii).Centroid(1)), idx2]));
    end
    %it = i;
end
for ii=1:size(allBlueBlobAreas,1)
    idx3 = subplus(round(allBlueBlobAreas(ii).Centroid(2)-allBlueBlobAreas(ii).MinorAxisLength*kdick)) +1;
    idx4 = subplus(round(allBlueBlobAreas(ii).Centroid(2)+allBlueBlobAreas(ii).MinorAxisLength*kdick)) +1;
    if idx4 > size(green,1)
        idx4 = size(green,1);
    end
    if (green(idx3, round(allBlueBlobAreas(ii).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allBlueBlobAreas(ii).Centroid(1)), idx3]));
    end
    if (green(idx4, round(allBlueBlobAreas(ii).Centroid(1))))
        greenBlobs = round(cat(1,greenBlobs,[round(allBlueBlobAreas(ii).Centroid(1)), idx4]));
    end
end
% One green point if in proximity.
greenBlobs = sort(greenBlobs,1);
if (size(greenBlobs,1)>1)
    tmp = [];
    last_green = [-100 -100];
    count = 1;
    for ii=1:(size(greenBlobs,1))
        if ~((abs(greenBlobs(ii,1)-last_green(1,1))<10)&&(abs(greenBlobs(ii,2)-last_green(1,2))<10))
            tmp(count,:)=greenBlobs(ii,:);
            count = count +1;
        end
        last_green = greenBlobs(ii,:);
    end
    greenBlobs = tmp;
end
% TODO: pick up small blue blobs - tempramental
% TODO: pick up small green

%% show images
% figure;imshow(chrom);
% for j=1:size(greenBlobs,1)
%     hold on; plot(greenBlobs(i,1),greenBlobs(i,2),'o--k')
% end
%% Keypoints of beacons
kp_list = [];
for it = 1:numel(allRedBlobAreas)
    kp_list = round([kp_list;allRedBlobAreas(it).Centroid, 1, allRedBlobAreas(it).MajorAxisLength]);
end
for it = 1:size(greenBlobs,1)
    kp_list = round([kp_list;[greenBlobs(it,1) greenBlobs(it,2)], 2, 0]); %Area for green not known - default 0.
end
for it = 1:numel(allBlueBlobAreas)
    kp_list = round([kp_list;allBlueBlobAreas(it).Centroid, 3, allBlueBlobAreas(it).MajorAxisLength]);
end

%% sort rows per pixel location of 1st column
if isempty(kp_list)
    z = [];
    return
end

kp_list = sortrows(kp_list,[1]);

%% Remove not complete beacon from kp_list
tolerance = 5;
%m1 = -0.0210896309314587;
%b1 = 1.5588466973637962;
%m2 = -0.142649154;
%b2 = 1.530568848;
m1 = -0.0359;
b1 = 2.0844;
m2 = -0.2923;
b2 = 3.5358;
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
            if ID ~= idReal(1) && ID ~= idReal(2) && ID ~= idReal(3) && ID ~= idReal(4) && ID ~= idReal(5)
                listA = [];
                continue
            end
            meanX = sum(listA(:,1))/3;
            meanY = sum(listA(:,2))/3;
            distY = listA(3,2)-listA(1,2);
            ran = m1*(distY)+b1;
            bearing = m2*((meanX-160)/ran)+b2;
            bearing = deg2rad(bearing);
            beacon = [ran+ 0.17, bearing, ID];
            z = cat(1,z,beacon);
            listB = cat(1,listB,listA);
            listA = [];
        end
    end
end
end
