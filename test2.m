[a,b]=hist(data.Global_Time,unique(data.Global_Time));
c=sortrows([a' b],1,'descend');
t = c(5,2);
t_min = c(n,2)-10*1000;
t_max = c(n,2)+10*1000;
t_sample = t+(0:Ts1:TT)*1000;
[veh_traj_set,frames]=data_interpolation(t_min,t_max,data);
for i=1:length(frames)
    affordance_set1{i} = calc_affordance(frames{i});
end


%% draw
for n = 2:20
duration = zeros(length(veh_traj_set),1);
for i=1:length(veh_traj_set)
        duration(i) = veh_traj_set(i).t(2);
end
[aa,bb]=sort(duration,'descend');
idx = bb(n);
lm = linspace(0,21.6,7);
f2m = 0.3048;


t0 = veh_traj_set(idx).t(1);
if t0<t_min
    t0 = t_min;
end
i0 = (t0-t_min)/100+3;

t1 = t0+veh_traj_set(idx).t(2);
if t1>t_max
    t1=t_max;
end
i1 = (t1-t_min)/100;

veh_ID = veh_traj_set(idx).Vehicle_ID;
for i=i0:i1
    figure(1)

clf
hold on
    entry = frames{i}(find(frames{i}.Vehicle_ID==veh_ID),:);
    ego_y = entry.Local_Y;
    ego_x = entry.Local_X;
    ego_v0 = entry.v_Vel;
    aff1 = table2array(affordance_set1{i}(affordance_set1{i}.Vehicle_ID==veh_ID,:));
    aff2 = table2array(affordance_set1{i-2}(affordance_set1{i-2}.Vehicle_ID==veh_ID,:));
    try
    a1 = aff1([2,3,4,6,7,9,10,11,12,14,15])'./x_norm';
    a2 = aff2([2,3,4,6,7,9,10,11,12,14,15])'./x_norm';
    pred = traj_predictor(a1,a2);
    
    
    
    for k=1:size(frames{i},1)
        if frames{i}.Vehicle_ID(k)==veh_ID
            draw_rec([frames{i}.Local_X(k),frames{i}.Local_Y(k)],[frames{i}.v_Width(k) frames{i}.v_length(k)],0,'r');
        elseif frames{i}.Local_Y(k)-ego_y>=-7 && frames{i}.Local_Y(k)-ego_y<=50
            draw_rec([frames{i}.Local_X(k),frames{i}.Local_Y(k)],[frames{i}.v_Width(k) frames{i}.v_length(k)],0,'b');
        end
    end
    for j=1:M
        if pred(j)>0
            plot(traj_base(j,m+1:2*m)+ego_x,ego_y+traj_base(j,1:m)+ego_v0*(0:m-1)*Ts1,'m--');
        end
    end
    possible_set = find(pred>0)
    
    for k=1:length(lm)
        plot([lm(k),lm(k)],[ego_y-7,ego_y+80],'r--')
    end
    axis([0,21.6,ego_y-7,ego_y+80])
    axis equal
    pause(0.05)
    end
end
end


