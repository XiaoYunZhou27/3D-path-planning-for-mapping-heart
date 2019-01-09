function [Mesh_out] = QEM_MS(Mesh_in,Tar_num, RATIO)
%% QEM_MS Quadric Error Metric Mesh Simplification
% Mesh_in(out).ver  3D coordinates of points
%             .nor  normal of points
%             .face triangulation connectivity
%             .cur  curvature of points
% Tar_num           target number of points
% RATIO             importance ratio of distance

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

%% Variable name conversion
Ver = Mesh_in.ver;
Nor = Mesh_in.nor;
Face = Mesh_in.face;
Cur = Mesh_in.cur;
%% Mesh simplification
tic;
for i = size(Mesh_in.ver,1):-1:Tar_num
    i
    % Decimatied ID
    [Inx,Edge,Tri] = Dec_id(Ver,Cur,Face,RATIO);
    Decid_1 = Edge(Inx(1,1),1);
    Decid_2 = Edge(Inx(1,1),2);
    Newpos = Decimation(Decid_1,Decid_2,Ver,Face,Nor); 
    % Update ver
    Ver_num = size(Ver,1);
    Ver = [Ver(1:Ver_num,:);Newpos];
    Ver(Decid_1,:) = [];
    Ver(Decid_2-1,:) = [];
    % Update Face
    Face_id = edgeAttachments(Tri,Edge(Inx(1,1),:));
    Face_id = cell2mat(Face_id);
    Face(min(Face_id),:) = [];
    Face(max(Face_id)-1,:) = [];
    [Row,Col] = find(Face == Decid_1 | Face == Decid_2);
    for j=1:size(Row)
        Face(Row(j,1),Col(j,1)) = Ver_num + 1;
    end
    [Row,Col] = find(Face > Decid_2);
    for j=1:size(Row)
        Face(Row(j,1),Col(j,1)) = Face(Row(j,1),Col(j,1)) - 1;
    end
    [Row,Col] = find(Face > Decid_1);
    for j=1:size(Row)
        Face(Row(j,1),Col(j,1)) = Face(Row(j,1),Col(j,1)) - 1;
    end
    % Update Cur
    Cur_out = (Cur(Decid_1,:) + Cur(Decid_2-1,:))/2;
    Cur(Decid_1,:) = [];
    Cur(Decid_2-1,:) = [];
    Cur = [Cur(1:Ver_num-2,:);Cur_out];
    % Update normal  
    Nor_out = (Nor(Decid_1,:) + Nor(Decid_2-1,:))/2;
    Nor(Decid_1,:) = [];
    Nor(Decid_2-1,:) = [];
    Nor = [Nor(1:Ver_num-2,:);Nor_out];
end

%% Variable name conversion
Mesh_out.ver= Ver;
Mesh_out.face = Face;
Mesh_out.cur = Cur;
Mesh_our.nor = Nor;
toc;
end