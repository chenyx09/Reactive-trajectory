%% generate training data
extract_data_NGSIM;

if ~exist('Bez_matr','var')
    bezier_regressor;
end
TT = 3;
Ts = 0.1;
Ts1 = 0.5;
t_traj = 0:Ts1:TT;
[a,b]=hist(data.Global_Time,unique(data.Global_Time));
c=sortrows([a' b],1,'descend');
T_min = min(b);
T_max = max(b);
processed_frames=[];
affordance_set = {};
counter = 0;
training_set=[];
traj_pool=[];
for n=1:300
    
    t0 = c(n,2);
    if n<=5||min(abs(t0-c(1:n-1,2)))>2000
        n
    t_min = c(n,2)-2*Ts1*1000;
    t_max = c(n,2)+1.5*TT*1000;
    t_sample = t0+(0:Ts1:TT)*1000;
    if t_min>T_min && t_max<T_max
        [veh_traj_set,frames]=data_interpolation(t_min,t_max,data);
        frame1 = frames{1};
        frame2 = frames{1+Ts1/Ts};
        frame3 = frames{1+2*Ts1/Ts};
        idx = find(processed_frames==frame1.Global_Time(1));
        if isempty(idx)
        processed_frames = [processed_frames frame1.Global_Time(1)];
        affordance_set{counter+1} = calc_affordance(frame1);
        affordance1 = affordance_set{counter+1};
        counter = counter+1;
        else
            affordance1 = affordance_set{idx};
        end
        idx = find(processed_frames==frame2.Global_Time(1));
        if isempty(idx)
        processed_frames = [processed_frames frame2.Global_Time(1)];
        affordance_set{counter+1} = calc_affordance(frame2);
        affordance2 = affordance_set{counter+1};
        counter = counter+1;
        else
            affordance2 = affordance_set{idx};
        end
        idx = find(processed_frames==frame3.Global_Time(1));
        if isempty(idx)
        processed_frames = [processed_frames frame3.Global_Time(1)];
        affordance_set{counter+1} = calc_affordance(frame3);
        affordance3 = affordance_set{counter+1};
        counter = counter+1;
        else
            affordance3 = affordance_set{idx};
        end
        for i=1:length(veh_traj_set)
            tt = veh_traj_set(i).t(1)+(0:100:veh_traj_set(i).t(2));
            dp.x_traj = interp1(tt,veh_traj_set(i).x_traj,t_sample);
            dp.y_traj = interp1(tt,veh_traj_set(i).y_traj,t_sample);
            dp.v_traj = interp1(tt,veh_traj_set(i).v_traj,t_sample);
            dp.x_traj = dp.x_traj-dp.x_traj(1);
            dp.y_traj = dp.y_traj-dp.y_traj(1);
            
            dp.affordance1=table2array(affordance1(affordance1.Vehicle_ID==veh_traj_set(i).Vehicle_ID,:));
            dp.affordance2=table2array(affordance2(affordance2.Vehicle_ID==veh_traj_set(i).Vehicle_ID,:));
            dp.affordance3=table2array(affordance3(affordance3.Vehicle_ID==veh_traj_set(i).Vehicle_ID,:));
            training_set=[training_set;dp];
            y_nom = veh_traj_set(i).v_traj(1)*(0:Ts1:TT);
            delta_y = dp.y_traj - y_nom;
            traj_pool = [traj_pool;delta_y dp.x_traj];
        end
%         dp.x_traj = interp
    end
    end
end

%% calculate adjacency matrix
m = TT/Ts1+1;
delta = 0.4;
[traj_base,N_cover]=traj_sparsification(traj_pool,m,delta);
% traj_base = traj_base(N_cover>5,:);
i=1;
while i<=size(traj_base,1)-1
    j=i+1;
    while j<=size(traj_base,1)
        if scaled_inf_norm(traj_base(i,:),traj_base(j,:))<delta
            if N_cover(i)>=N_cover(j)
                traj_base(j,:)=[];
                N_cover(j)=[];
            else
                traj_base(i,:)=[];
                N_cover(i)=[];
                i=i-1;
                break
            end
        else
            j=j+1;
        end
    end
    i=i+1;
end
                
%% remove similar trajectories

%% draw example trajectory
idx = [1,2,3,4,5];
figure(1)
clf
hold on
% v0 = traj_pool(idx,2*m+1);
for n = 1:length(idx)

v0 = 15;
y_traj = traj_base(idx(n),1:m)+v0*(0:Ts1:TT);
x_traj = traj_base(idx(n),m+1:2*m);
cmap = {'r','b','c','m','g'};
for i = 1:m
    
    ellipse(sqrt(delta)/y_scaling(i),sqrt(delta)/x_scaling(i),0,y_traj(i),x_traj(i),cmap{n});
end

axis equal

end

%% draw trajectory base
figure(2)
clf
hold on
v0=15;
for i=1:size(traj_base,1)
    y_traj = traj_base(i,1:m)+v0*(0:Ts1:TT);
    x_traj = traj_base(i,m+1:2*m);
    plot(x_traj,y_traj);
end
plot([-1.8,-1.8],[-3,60]);
plot([1.8,1.8],[-3,60]);
axis equal
