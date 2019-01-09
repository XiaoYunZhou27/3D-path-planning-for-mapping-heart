function [Q_geo] = Qgeo_calculation(Ver_inx,Ver,Ver_norm,Nei_num)
%% Calculation of Quadric Martrics
% Input:  Ver_inx   index of point
%         Ver       3D coordinates of points
%         Ver_norm  normal of points
%         Nei_num   number of points seen as connected by edges
% Output: Q_geo     calculated Quadric Martrics

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

[Nei, ~] = knnsearch(Ver,Ver(Ver_inx,:),'k',Nei_num,'NSMethod','euclidean');
Q_geo = zeros(4,4);
for i=2:Nei_num
    inx_tmp = Nei(1,i);
    e_vec = Ver(inx_tmp,1:3) - Ver(Ver_inx,1:3);
    b_vec = cross(Ver_norm(Ver_inx,:),e_vec);
    norm_vec = cross(e_vec,b_vec);
    norm_vec = norm_vec/norm(norm_vec);
    D_tmp = dot(norm_vec,Ver(inx_tmp,1:3));
    P_vec = [norm_vec D_tmp];
    Squa_P = P_vec'*P_vec;
    Q_geo = Q_geo + Squa_P;
end

end