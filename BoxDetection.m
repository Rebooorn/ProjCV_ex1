close all;
clear all;
clc;

load('example1kinect.mat');
% load('example2kinect.mat');
% load('example3kinect.mat');
% load('example4kinect.mat');

% Display 3D point cloud for nothing.
% 
% c1 = cloud1(:,:,1);
% c2 = cloud1(:,:,2);
% c3 = cloud1(:,:,3);
% 
% c1 = reshape(c1,[424*512,1]);
% c2 = reshape(c2,[424*512,1]);
% c3 = reshape(c3,[424*512,1]);
% 
% c1 = downsample(c1,19);
% c2 = downsample(c2,19);
% c3 = downsample(c3,19);
% 
% scatter3(c1,c2,c3);

cloud = cloud1;

%% RanSAC algorithm
max_inlier = 0;
best_model = planeModel;
best_model.planeVector = [0 0 0];
best_model.maxIteration = 10000;
best_model.threshold = floor(0.6*424*512);
delta = 0.05;
% model contains: 1. plane parameter(3D vector);
%                 2. threshold to measure the quality of the model (use
%                 60% of pixel num here)
%                 3. parameter for maximum iterations
count = 0;
while count<best_model.maxIteration
%   randomly sample 3 points for model calculation
%   random sample should avoid unsuccessful points
    samplesX=[1,1,1];
    samplesY=[1,1,1];
    while 
        samplesX = randsample(size(cloud,1),3);
        samplesY = randsample(size(cloud,2),3);
    end
    model = estimate_plane(samplesX,samplesY,best_model,cloud);
    inlier = computeInlier(cloud,model,delta); % return binary image
    num_inlier = length(inlier(inlier==1));
    if num_inlier>max_inlier
        best_model = model;
        max_inlier = num_inlier;
    end
%   exit rule
    if num_inlier > best_model.threshold
        break;
    end
end
% mask amplitudes image and display
floor = computeInlier(cloud,best_model,delta);
figure; imagesc(floor); title('floor detected');









%%



