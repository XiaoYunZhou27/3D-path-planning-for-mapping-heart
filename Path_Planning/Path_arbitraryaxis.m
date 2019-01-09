function [Vid_out] = Path_arbitraryaxis(Vmap,Ver,Axis_id)
% Path planning along arbitrary axis
% Input:   Vmap     coordinates of conformal mapping
%          Ver      coordinates of vetexes
%          Axis_id  index of vetexes defining axis
% Output:  Vid_out  the order index of vertexes that the path goes through

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

N_LONGTITUDE = 9;
%% Midpoint of axis
V_sta = Vmap(Axis_id(1,1),:);
V_end = Vmap(Axis_id(1,2),:);
V_mid = (V_sta + V_end)/2;
V_mid = V_mid/norm(V_mid);

%% Turn around the spere,the arbitrary axis is parallel with X axis now
[Azi,E1e,~] = cart2sph(V_mid(1,1),V_mid(1,2),V_mid(1,3));
Tran_mar_z = [cos(Azi) -sin(Azi) 0;sin(Azi) cos(Azi) 0;0 0 1];
Tran_mar_y = [cos(E1e) 0 -sin(E1e);0 1 0;sin(E1e) 0 cos(E1e)];
Vmap = Vmap*Tran_mar_z*Tran_mar_y;
Ang = atan2(Vmap(Axis_id(1,1),2),Vmap(Axis_id(1,1),3));
Tran_mar_x = [1 0 0;0 cos(Ang) sin(Ang);0 -sin(Ang) cos(Ang)];
Vmap = Vmap*Tran_mar_x;
V_mid = V_mid*Tran_mar_z*Tran_mar_y*Tran_mar_x;

%% Translate sphere
Dis = Vmap(Axis_id(1,1),1); 
Vmap(:,1) = Vmap(:,1) - Dis;

%% Longtitude and latitude partition
[Azi,Ele,~] = cart2sph(Vmap(:,1),Vmap(:,2),Vmap(:,3));
[Row,~] = find(Azi<0 & Azi>=-1*pi);
Azi(Row,:) = Azi(Row,:) + 2*pi;

[Row,~] = find(Ele>=0 & Ele<0.5*pi);
Ele(Row,:) = 0.5*pi - Ele(Row,:);
[Row,~] = find(Ele>=0.5*pi & Ele<pi);
Ele(Row,:) = Ele(Row,:) - 0.5*pi;
[Row,~] = find(Ele>=-1*pi & Ele<-0.5*pi);
Ele(Row,:) = Ele(Row,:) + 1.5*pi;
[Row,~] = find(Ele>=-0.5*pi & Ele<0);
Ele(Row,:) = 0.5*pi - Ele(Row,:);
Ele = -Ele;
% %% Path planning
% Vid_out = zeros(size(Vmap,1),1);
% Vid_out(1,1) = Axis_id(1,1);
% Int = 2*pi/N_LONGTITUDE;
% for i=1:N_LONGTITUDE
%     [Row,~] = find(Azi>=(i-1)*Int & Azi<i*Int);
%     [Vstaid,~] = find(Vid_out==0,1);
%     Vsta = Vmap(Vstaid-1,:);
%     Ver_sub = Ver(Row,:); 
%     spid = ShortPath(Ver_sub,Vsta);
%     for j=1:size(spid,1)
%         Vid_out(Vstaid-1+j,1) = Row(j,1);
%     end
% end
%% Path planning
Vid_out = zeros(size(Ver,1),1);
Int = 2*pi/N_LONGTITUDE;
Count = 0;
for i=1:N_LONGTITUDE
    [Row,~] = find(Azi>=(i-1)*Int & Azi<i*Int);
    Sub_ele = Ele(Row,1);
    if mod(i,2)==1
        [~,Id] = sort(Sub_ele,1);
        for j=1:size(Row,1)
            Vid_out(Count+j) = Row(Id(j,1),1);
        end
    else
        [~,Id] = sort(Sub_ele,1,'descend');
        for j=1:size(Row,1)
            Vid_out(Count+j) = Row(Id(j,1),1);
        end
    end
    Count = Count + size(Row,1);
end
end
