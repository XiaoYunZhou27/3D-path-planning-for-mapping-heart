function [PC_out] = QEM_PC(PC_in,Tar_num,RATIO)
%% QEM_PC Quadric Error Metric Point Cloud Simplification
% PC_in(out).ver     3D coordinates of points
%           .nor     normal of points
%           .cur     curvature of points
%           Tar_num  target number of points
%           RATIO    importance ratio of distance

%% Point Cloud Simplification
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

%% Parameters
Nei_num = 16;% the number of neighbours for a point
Nei_num_gau = 5;% the number of neighbours for updating curvature/normal

%% Variable name conversion
Ver = PC_in.ver;
Nor = PC_in.nor;
Cur = PC_in.cur;

%% Point Cloud simplification
for i = size(PC_in.ver,1):-1:Tar_num+1
    i
    % Decide the two points to be decimated
    [Decid_1,Decid_2] = Dec_id(Ver,Cur,RATIO,Nei_num);
    % Decimate to the new posigion
    New_pos = Decimation(Decid_1,Decid_2,Ver,Nor,Nei_num);
    New_pos = -New_pos;
    % update Ver
    Ver_num = size(Ver,1);
    Ver = [Ver(1:Ver_num,:);New_pos(1:3,1)'];
    % update curvature and normal
    New_cur = Gaussian_wei_cur(Ver_num+1,Ver,Cur,Nei_num_gau);
    New_nor = Gaussian_wei_norm(Ver_num+1,Ver,Nor,Nei_num_gau);
    Ver(Decid_1,:) = [];
    Ver(Decid_2-1,:) = [];
    Cur = [Cur(1:Ver_num,:);New_cur]; 
    Nor = [Nor(1:Ver_num,:);New_nor];
    Cur(Decid_1,:) = [];
    Cur(Decid_2-1,:) = [];
    Nor(Decid_1,:) = [];
    Nor(Decid_2-1,:) = [];
end

%% Variable name conversion
PC_out.ver = Ver;
PC_out.nor = Nor;
PC_out.cur = Cur;
end