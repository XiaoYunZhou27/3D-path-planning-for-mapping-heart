function [Cur_out] = Gaussian_wei_cur(Ver_inx,Ver,Cur_in,Nei_num)
%% Calculate Gassian Interpolated curvature for new vertex
%Input:  Ver_inx   index of point
%        Ver      3D coordinates of point
%        Cur_in    curvature of all points
%        Nei_num   number of points seen as connected
%Output: Cur_out   curvature for new vertex

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

Cur_out = 0;
Nei = knnsearch(Ver,Ver(Ver_inx,:),'k',Nei_num,'NSMethod','euclidean');
h = mean(Nei);
sum = 0;
for i=2:Nei_num
    Nei_inx = Nei(1,i);
    if Nei_inx<Ver_inx
        Cur_out = Cur_out + Cur_in(Nei_inx,:)*exp(norm(Ver(Ver_inx,:)-Ver(Nei_inx,:))/h^2);
        sum = sum + exp(norm(Ver(Ver_inx,:)-Ver(Nei_inx,:))/h^2);
    end
end
Cur_out = Cur_out/sum;
end