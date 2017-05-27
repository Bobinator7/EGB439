clear;close all;clc;

global m1
global m2
global b1 
global b2

% m1 = -0.04182;
% b1 = 2.0865;
% m2 = -0.28628;
% b2 = 2.6613;

m1 = -0.0359;
b1 = 2.0844;
m2 = -0.2923;
b2 = 3.5358;

global img
global z

img = getImage;
%cd ../../..
%img = imread('beaconImage250.jpg');
imshow(img);
%img = pb.getImageFromCamera();
%img = imrotate(img,-90);

%z = senseless(img)

A = slidevar('m1',[-0.1,-0.02]);
B = slidevar('m2',[-0.5,-0.2]);
C = slidevar('b1',[1.4,3]);
D = slidevar('b2',[1.9,5]);