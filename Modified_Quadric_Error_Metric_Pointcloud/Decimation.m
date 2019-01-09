function [V_new] = Decimation(V1_inx, V2_inx,Ver,Nor,Nei_num)
%% Derterming new vertex position
% Input:  V1_inx   one index of decimation point
%         V2_inx   the other index of decimation point
%         Ver      3D coordinates of points
%         Nor      normal of points
%         Nei_num  number of points seen as connected
% Output: V_new    new position after decimation

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

Q1_geo = Qgeo_calculation(V1_inx,Ver,Nor,Nei_num);
Q2_geo = Qgeo_calculation(V2_inx,Ver,Nor,Nei_num);
Q_aver = (Q1_geo+Q2_geo)/2;
Q_tmp = [Q_aver(1,1:4);Q_aver(1,2) Q_aver(2,2:4);Q_aver(1,3) Q_aver(2,3) Q_aver(3,3:4);0 0 0 1];
if abs(det(Q_tmp))<10^-20
    V_new = (Ver(V1_inx,1:3)+Ver(V2_inx,1:3))/2;
else
    V_new = inv(Q_tmp)*[0 0 0 1]';
end
end