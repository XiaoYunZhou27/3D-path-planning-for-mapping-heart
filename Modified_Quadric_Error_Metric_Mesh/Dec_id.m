function [Inx,Edge,Tri] = Dec_id(Ver,Cur,Face,RATIO)
%% Derterming the vanishing queue
% Input:  Ver     coordinates of points
%         Cur     curvature of points
%         Face    connectivity of mesh
%         RATIO   importance ratio of distance

% Output: Inx     vanishing queue of edges
%         Edge    edge connection in triangulation class
%         Tri     triangulation connection in triangulation class

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

Tri = triangulation(Face,Ver);
Edge = edges(Tri);
Dis = zeros(size(Edge,1),1);
Cur_jud = zeros(size(Edge,1),1);
for i=1:size(Edge,1)
    Dis(i,1) = norm(Ver(Edge(i,1),:) - Ver(Edge(i,2),:));
    Cur_jud(i,1) = Cur(Edge(i,1),:) + Cur(Edge(i,2),:);
end
% Normalise distance
Dis = Dis/max(Dis);
% Normalise curvature
Cur_jud = Cur_jud/max(Cur_jud);
Jud = Dis*RATIO + Cur_jud*(1-RATIO);
[~,Inx] = sort(Jud);
end