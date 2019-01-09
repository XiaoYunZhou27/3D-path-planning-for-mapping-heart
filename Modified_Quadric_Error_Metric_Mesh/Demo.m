%% Modified_Quadric_Error_Metric_Mesh
%  Simplify high resolution meshes to low resolution one (target number of points) with distribution control and details preservation
%  This is the demo, for funtions, please see other files

% ParallaxBA: by Xiao-Yun Zhou(xiaoyun.zhou14@imperial.ac.uk), Sabine Ernst, Su-Lin Lee, Modified_Quadric_Error_Metric_Mesh is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
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
Tar_num = 100;
% Importance ratio of distance
RATIO = 0.5;

load('TestData.mat');
% Simplification
Mesh_sim = QEM_MS(TestData,Tar_num,RATIO);

RV = TestData;
RV_sim = Mesh_sim;
save('RV.mat','RV');
save('RV_sim.mat','RV_sim');

figure;
subplot(1,2,1);
patch('Faces',TestData.face,'Vertices',TestData.ver,'FaceColor','w','EdgeColor',[0.3 0.3 0.3],'EdgeAlpha',1);
title('Original Mesh');
axis off;
subplot(1,2,2);
patch('Faces',Mesh_sim.face,'Vertices',Mesh_sim.ver,'FaceColor','w','EdgeColor',[0.3 0.3 0.3],'EdgeAlpha',1);
title('Simplified Mesh');
axis off;