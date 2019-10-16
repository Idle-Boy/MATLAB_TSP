clear;close all;
%% ���г�ʼ��
N = 13;
Citys = round( rand (N,2) * 100 );
D = zeros(N,N);     % ��ų��м����
for i = 1 : N
    for j = 1 : N
        D(i,j) = sqrt(sum((Citys(i,:) - Citys(j,: )).^2));
    end
end

%% ���ӳ�ʼ��
G = 5000;     % ��������
iter = 1;
Iter_max = 1000;    % ����������
% C1 = 0.3;   % ����ѧϰ����
% C2 = 0.5;   % Ⱥ��ѧϰ����
% W = 1;      % ����Ȩ��
Length = zeros(G,1);    %��¼ÿ�����ӵõ������е�·������
route = zeros(G,N);

for i = 1 : G       %���Ϊ���Ӹ���ʼ������
    route(i,:) = randperm(N);
end
routebest = route;  %���ڼ�¼�������ӵõ�������·��
% for i = 1 : G
%     for j = 1 : (N-1)
%         Length(i) = Length(i) + D( RouteBest(i,j),RouteBest(i,j+1) );
%     end
%     Length(i) = Length(i) + D( RouteBest(i,N),RouteBest(i,1));
% end
% [ GLength,index ] = min(Length);
% GRouteBest = RouteBest (index,:);
%% ��ʼ����ʼ�ٶ�
IniExSeq_V = zeros(N-1,2,G);
for i = 1 : G
    IniExSeq_V(:,1,i)= randperm(N-1);
    IniExSeq_V(:,2,i)= randperm(N-1);
end
%% �����ʼ���еĸ���·�����ȣ����ҳ�ȫ�����Ÿ�ֵ�� GRouteBest
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
    %% ������·����ȫ������·�����ֲ�����·���Աȣ���¼ȫ�ֽ����ӡ��ֲ�������
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
    %% �����������ֽ����ӣ�����PSO�ٶȸ��¹�ʽ���Գ�ʼ������ͽ����Ӱ�������ϣ�ÿ�����Ӷ����£�
    tempV = zeros(2*(N-1),2,G);
    for i = 1 : G
        tempV(1:12,:,i) = IniExSeq_V(1:12,:,i);
        tempV(13:24,:,i) = [ExSeq_Vg(1:8,:,i);ExSeq_Vp(1:4,:,i)];
    end
    %% �����������µõ��Ĺ�ʽ����ÿ�����ӵĳ�ʼ·������ϸ��º�Ľ�������,����·������
    route_temp = route;                 % ��¼����֮ǰ��·�������ڸ��º������������������
    for i = 1 : G
        for j = 1:2*(N-1)
            if tempV(j,1,i) ~= 0
                temp = route(i,tempV(j,1,i));
                route(i,tempV(j,1,i)) = route(i,tempV(j,2,i));
                route(i,tempV(j,2,i)) = temp;
            end
        end
    end
    %% �������·����·���ĳ���
    Length2 = zeros(G,1);
    for i = 1 : G
        for j = 1 : (N-1)
            Length2(i) = Length2(i) + D( route(i,j),route(i,j+1) );
        end
        Length2(i) = Length2(i) + D( route(i,N),route(i,1));
    end
    %% ���¾ֲ�����·��
    for i = 1 : G
        if (Length2(i) <= Length(i))
            routebest(i,:) = route(i,:);
            Length(i) = Length2(i);
        end
    end
    %% ����ȫ������·����ȫ������·��ֵ
    if GLength >= min(Length2)
        [ GLength , index ] = min(Length2);
        GRouteBest = route (index,:);
    end
    %% ����·������ǰ������б�ţ�����������������У����� IniExSeq_V (δ�����������)
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
