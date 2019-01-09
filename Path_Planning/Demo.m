%% Path_Planning
% This demonstrates how to plan a path that goes through a mesh along a
% certain axis. For calculating the conformal mapping of a mesh, please go
% to http://www.lokminglui.com/software.html
% Please cite: "K.C. Lam, P.T. Choi and L.M. Lui, FLASH: Fast landmark 
% aligned spherical harmonic parameterization for genus-0 closed brain surfaces, 
%SIAM Journal on Imaging Sciences, 8(1), 67-94 (2015)" when using this software tool.

% ParallaxBA: by Xiao-Yun Zhou(xiaoyun.zhou14@imperial.ac.uk), Sabine Ernst, Su-Lin Lee, Path_Planning is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
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
clear all;
close;

load('RV_sim.mat');
load('RV_sim_con.mat');
% Index of points defining the axis
Axi_id = [24,55];
% Path Planning
[Sim.pid] = Path_arbitraryaxis(RV_sim_con.ver ,RV_sim.ver,Axi_id);

figure;
patch('Faces',RV_sim.face,'Vertices',RV_sim_con.ver,'FaceColor','w','EdgeColor',[0.3 0.3 0.3],'FaceAlpha',0.5,'EdgeAlpha',0.5);
hold on;
plot3(RV_sim_con.ver(Sim.pid,1),RV_sim_con.ver(Sim.pid,2),RV_sim_con.ver(Sim.pid,3),'r.-','MarkerSize',20,'LineWidth',2);
hold on;
plot3(RV_sim_con.ver(Axi_id,1),RV_sim_con.ver(Axi_id,2),RV_sim_con.ver(Axi_id,3),'k.-','MarkerSize',30,'LineWidth',5);
hold off;
axis off;
title('Path Planning on Conformal Mapping');
figure;
patch('Faces',RV_sim.face,'Vertices',RV_sim.ver,'FaceColor','w','EdgeColor',[0.3 0.3 0.3],'FaceAlpha',0.5,'EdgeAlpha',0.5);
hold on;
plot3(RV_sim.ver(Sim.pid,1),RV_sim.ver(Sim.pid,2),RV_sim.ver(Sim.pid,3),'r.-','MarkerSize',20,'LineWidth',2);
hold on;
plot3(RV_sim.ver(Axi_id,1),RV_sim.ver(Axi_id,2),RV_sim.ver(Axi_id,3),'k.-','MarkerSize',30,'LineWidth',5);
hold off;
axis off;
title('Path Planning on Simplified Mesh');