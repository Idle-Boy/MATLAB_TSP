clear;close all;
%% 城市初始化
N = 13;
Citys = round( rand (N,2) * 100 );
D = zeros(N,N);     % 存放城市间距离
for i = 1 : N
    for j = 1 : N
        D(i,j) = sqrt(sum((Citys(i,:) - Citys(j,: )).^2));
    end
end

%% 粒子初始化
G = 5000;     % 粒子数量
iter = 1;
Iter_max = 1000;    % 最大迭代次数
% C1 = 0.3;   % 自我学习因子
% C2 = 0.5;   % 群体学习因子
% W = 1;      % 惯性权重
Length = zeros(G,1);    %记录每个粒子得到的序列的路径长度
route = zeros(G,N);

for i = 1 : G       %随机为粒子赋初始化序列
    route(i,:) = randperm(N);
end
routebest = route;  %用于记录各个粒子得到的最优路径
% for i = 1 : G
%     for j = 1 : (N-1)
%         Length(i) = Length(i) + D( RouteBest(i,j),RouteBest(i,j+1) );
%     end
%     Length(i) = Length(i) + D( RouteBest(i,N),RouteBest(i,1));
% end
% [ GLength,index ] = min(Length);
% GRouteBest = RouteBest (index,:);
%% 初始化初始速度
IniExSeq_V = zeros(N-1,2,G);
for i = 1 : G
    IniExSeq_V(:,1,i)= randperm(N-1);
    IniExSeq_V(:,2,i)= randperm(N-1);
end
%% 计算初始序列的各个路径长度，并找出全局最优赋值给 GRouteBest
for i = 1 : G
    for j = 1 : (N-1)
        Length(i) = Length(i) + D( route(i,j),route(i,j+1) );
    end
    Length(i) = Length(i) + D( route(i,N),route(i,1));
end
[ GLength , index ] = min(Length);
GRouteBest = route (index,:);   
% ExSeq_Vp = zeros(N-1,2,G);
% ExSeq_Vg = zeros(N-1,2,G); 
while iter < Iter_max
%     for i = 1 : G
%         for j = 1 : (N-1)
%             Length(i) = Length(i) + D( Route(i,j),Route(i,j+1) );
%         end
%         Length(i) = Length(i) + D( Route(i,N),Route(i,1));
%     end
%     [ GLength , index ] = min(Length);
%     GRouteBest = Route (index,:);
    ExSeq_Vp = zeros(N-1,2,G);
    ExSeq_Vg = zeros(N-1,2,G);
    %% 将所有路径与全局最优路径、局部最优路径对比，记录全局交换子、局部交换子
    for i = 1 : G
        temp1 = route(i,:);
        temp2 = temp1;
        for j = 1 : N-1
            if temp1(j) ~= GRouteBest(j)
                Loc = find (temp1 == GRouteBest(j));
                ExSeq_Vg(j,1,i) = j;
                ExSeq_Vg(j,2,i) = Loc;
                Temp_n = temp1(j);
                temp1(j) = temp1(Loc);
                temp1(Loc) = Temp_n;
            end
            if iter > 1
                if temp2(j) ~=  routebest(i,j);
                    loc = find (temp2 == routebest(i,j));
                    ExSeq_Vp(j,1,i) = j;
                    ExSeq_Vp(j,2,i) = Loc;
                    temp_n = temp2(j);
                    temp2(j) = temp2(Loc);
                    temp2(Loc) = temp_n;
                end
            end
        end
    end
    %% 根据上述两种交换子，仿照PSO速度更新公式，对初始交换序和交换子按比例组合（每个粒子都更新）
    tempV = zeros(2*(N-1),2,G);
    for i = 1 : G
        tempV(1:12,:,i) = IniExSeq_V(1:12,:,i);
        tempV(13:24,:,i) = [ExSeq_Vg(1:8,:,i);ExSeq_Vp(1:4,:,i)];
    end
    %% 根据上述更新得到的公式，对每个粒子的初始路径，结合更新后的交换序列,进行路径更新
    route_temp = route;                 % 记录更新之前的路径，便于更新后整理出基本交换序列
    for i = 1 : G
        for j = 1:2*(N-1)
            if tempV(j,1,i) ~= 0
                temp = route(i,tempV(j,1,i));
                route(i,tempV(j,1,i)) = route(i,tempV(j,2,i));
                route(i,tempV(j,2,i)) = temp;
            end
        end
    end
    %% 计算更新路径后，路径的长度
    Length2 = zeros(G,1);
    for i = 1 : G
        for j = 1 : (N-1)
            Length2(i) = Length2(i) + D( route(i,j),route(i,j+1) );
        end
        Length2(i) = Length2(i) + D( route(i,N),route(i,1));
    end
    %% 更新局部最优路径
    for i = 1 : G
        if (Length2(i) <= Length(i))
            routebest(i,:) = route(i,:);
            Length(i) = Length2(i);
        end
    end
    %% 更新全局最有路径、全局最优路径值
    if GLength >= min(Length2)
        [ GLength , index ] = min(Length2);
        GRouteBest = route (index,:);
    end
    %% 依据路径更新前后的序列编号，整理出基本交换序列，赋给 IniExSeq_V (未完待续！！！)
    IniExSeq_V = zeros(N-1,2,G);    
    for i = 1 : G
        temp1 = route_temp(i,:);
        for j = 1 : N-1
            if temp1(j) ~= route(i,j)
                loc = find (temp1 == route(i,j));
                IniExSeq_V(j,1,i) = j;
                IniExSeq_V(j,2,i) = loc;
                Temp_n = temp1(j);
                temp1(j) = temp1(loc);
                temp1(loc) = Temp_n;
            end
        end
    end
    iter = iter + 1;
end
plot([Citys(GRouteBest,1);Citys(GRouteBest(1),1)],[Citys(GRouteBest,2);Citys(GRouteBest(1),2)],'o-')
