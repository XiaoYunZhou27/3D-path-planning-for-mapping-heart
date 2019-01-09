function [Q_geo] = Qgeo_calculation(Vid,Ver,Face,Nor)
%% Calculation of Quadric Martrics
% Input: Vid    index of points
%        Ver    3D coordinates of points
%        Face   triangulation faces
%        Nor    normals of points

%Output: Q_geo  quadric martrics of points

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

%% Neighbours of Vid(Nei)
Tri = triangulation(Face,Ver);
Edge = edges(Tri);
[Row,Col] = find(Edge == Vid);
[tmpR_1,~] = find(Col == 1);
Col(tmpR_1,:) = 0;
[tmpR_2,~] = find(Col == 2);
Col(tmpR_2,:) = 1;
Col(tmpR_1,:) = 2;
for i=1:size(Row)
    Nei(i,1) = Edge(Row(i,1),Col(i,1));
end
%% Q calculation
Q_geo = zeros(4,4);
for i=1:size(Nei,1)
    Id = Nei(i,1);
    e_vec = Ver(Id,:) - Ver(Vid,:);
    b_vec = cross(Nor(Vid,:),e_vec);
    norm_vec = cross(e_vec,b_vec);
    norm_vec = norm_vec/norm(norm_vec);
    D_tmp = dot(norm_vec,Ver(Id,1:3));
    P_vec = [norm_vec D_tmp];
    Squa_P = P_vec'*P_vec;
    Q_geo = Q_geo + Squa_P;
end
end