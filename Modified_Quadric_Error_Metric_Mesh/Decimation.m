function [Newpos] = Decimation(Decid_1, Decid_2,Ver,Face,Nor)
%% Derterming new vertex position
% Input:  Decid_1  one index of decimation point
%         Decid_2  the other index of decimation point
%         Ver      3D coordinates of points
%         Face     triangulation connectivity
%         Nor      normal of points
% Output: Newpos   new position after decimation

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

Q1_geo = Qgeo_calculation(Decid_1,Ver,Face,Nor);
Q2_geo = Qgeo_calculation(Decid_2,Ver,Face,Nor);
Q_aver = (Q1_geo+Q2_geo)/2;
Q_tmp = [Q_aver(1,1:4);Q_aver(1,2) Q_aver(2,2:4);Q_aver(1,3) Q_aver(2,3) Q_aver(3,3:4);0 0 0 1];
if isnan(Q_tmp)==0
    if abs(det(Q_tmp))<10^-15
        Newpos = (Ver(Decid_1,1:3)+Ver(Decid_2,1:3))/2;
    else
        Newpos = inv(Q_tmp)*[0 0 0 1]';
        Newpos = -Newpos(1:3,1)';
    end
else 
    Newpos = (Ver(Decid_1,1:3)+Ver(Decid_2,1:3))/2;
end
end