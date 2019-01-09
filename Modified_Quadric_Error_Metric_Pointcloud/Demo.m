%% Modified_Quadric_Error_Metric_Pointcloud
%  Simplify high resolution point cloud to low resolution one (target number of points) with distribution control and details preservation
%  This is the demo, for funtions, please see other files

% ParallaxBA: by Xiao-Yun Zhou(xiaoyun.zhou14@imperial.ac.uk), Sabine Ernst, Su-Lin Lee, Modified_Quadric_Error_Metric_Pointcloud is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
% 
% 1) You can freely use and modify this code.
% 
% 2) If you want to distribute code based on this one, it has to be done under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
% 
% If you use this code for academic work, please reference:
% 
%       Xiao-Yun Zhou, Sabine Ernst and Su-Lin Lee,
%       Path Planning for Robot-Enhanced Cardiac Radiofrequency Catheter Ablation,
%       IEEE International Conference on Robotics and Automation (ICRA), 2016.
%      
% 
% 3) For commercial use, please contact the authors.

clc;
close all;
clear;

% The target number of points
Tar_num = 200;
% Importance ratio of distance
RATIO = 0.3;

load('TestData.mat');
Mesh_sim = QEM_PC(TestData,Tar_num,RATIO);

figure;
subplot(1,2,1);
scatter3(TestData.ver(:,1),TestData.ver(:,2),TestData.ver(:,3),'.k');
title('Original Mesh');
axis off;
subplot(1,2,2);
scatter3(Mesh_sim.ver(:,1),Mesh_sim.ver(:,2),Mesh_sim.ver(:,3),'.k');
title('Simplified Mesh');
axis off;