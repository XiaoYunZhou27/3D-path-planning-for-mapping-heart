function [Decid_1,Decid_2] = Dec_id(Ver,Cur,RATIO,Nei_num)
%% Derterming the vanishing queue
% Input:  Ver      3D coordinates of points
%         Cur      curvature of points
%         RATIO    importance ratio of distance
%         Nei_num  number of points seen as connected
% Output: Decid_1  one index of decimation point
%         Decid_2  the other index of decimation point

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

%% Define Decid_1
[~,Dis] = knnsearch(Ver,Ver,'k',Nei_num+1);
Dis = mean(Dis(:,2:Nei_num+1),2);
Cur = Cur/max(Cur);
Dis = Dis/max(Dis);
Que = RATIO*Dis + (1-RATIO)*Cur;
[~,Decid_1] = min(Que);
%% Define Decid_2
[Nei,Dis] = knnsearch(Ver,Ver(Decid_1,:),'k',Nei_num+1);
Dis = Dis(1,2:Nei_num+1)/max(Dis(1,2:Nei_num+1));
Dis = Dis';
Cur = Cur(Nei(1,2:Nei_num+1),1);
Que = RATIO*Dis + (1-RATIO)*Cur;
[~,Id] = min(Que);
Decid_2 = Nei(1,Id+1);

%% Sort decimated id
Min = min([Decid_1,Decid_2]);
Max = max([Decid_1,Decid_2]);
Decid_1 = Min;
Decid_2 = Max;
end