function [veh_traj_set,frames]=data_interpolation(t_min,t_max,data)
global N_bezier
Ts=0.1;
f2m = 0.3048;
seg = data(data.Global_Time>=t_min-2000&data.Global_Time<=t_max+2000,:);
vehicle_set = unique(seg.Vehicle_ID);
Vehicle_ID=[];
Local_X=[];
Local_Y=[];
v_Class=[];
v_Length=[];
v_Width=[];
v_Vel=[];
v_Acc=[];
Lane_ID=[];
veh_traj.Vehicle_ID = [];
veh_traj.t = [];
veh_traj.x_traj = [];
veh_traj.y_traj = [];
veh_traj.v_traj = [];
veh_traj.acc_traj = [];
veh_traj_set = [veh_traj];
counter = 1;

for i=1:(t_max-t_min)/100+1
%     frames{i}=table(Vehicle_ID,Local_X,Local_Y,v_Class,v_Length,v_Width,v_Vel,v_Acc,Lane_ID);
    frames{i}=[];
end
for i=1:length(vehicle_set)
seg1 = seg(seg.Vehicle_ID == vehicle_set(i),:);
seg1 = sortrows(seg1,'Global_Time');
[~,idx]=unique(seg1.Frame_ID);
seg1 = seg1(idx,:);
T = (seg1.Global_Time(end)-seg1.Global_Time(1))/1000;
tt = 0:0.1:T;
t_idx = (seg1.Global_Time-seg1.Global_Time(1))/100;


x_traj = seg1.Local_X*f2m;
y_traj = seg1.Local_Y*f2m;
if length(x_traj)>2*N_bezier
[x_bar,~] = bezier_regression_v1(x_traj,t_idx,tt);
[y_bar,v1] = bezier_regression_v1(y_traj,t_idx,tt);
% [v_bar,a1] = bezier_regression_v1(seg1.v_Vel*f2m,t_idx,tt);
v_bar = interp1((seg1.Global_Time-seg1.Global_Time(1))/1000,seg1.v_Vel*f2m,tt);
[acc_bar,~] = bezier_regression_v1(seg1.v_Acc*f2m,t_idx,tt);
lane_ID = round((x_bar+1.8)/3.6);
lane_ID = max(lane_ID,1);
lane_ID = min(lane_ID,6);
v_bar = max(0,v_bar);
elseif length(x_traj)>2
    x_bar = interp1((seg1.Global_Time-seg1.Global_Time(1))/1000,x_traj,tt);
    y_bar = interp1((seg1.Global_Time-seg1.Global_Time(1))/1000,y_traj,tt);
    v_bar = interp1((seg1.Global_Time-seg1.Global_Time(1))/1000,seg1.v_Vel*f2m,tt);
    acc_bar = interp1((seg1.Global_Time-seg1.Global_Time(1))/1000,seg1.v_Acc*f2m,tt);
    lane_ID = round((x_bar+1.8)/3.6);
    lane_ID = max(lane_ID,1);
    lane_ID = min(lane_ID,6);
    v_bar = max(0,v_bar);
else
    tt=[];
    T=0;
end

if seg1.v_Class(1)==2 && seg1.Global_Time(1)<=t_min && seg1.Global_Time(end)>=t_max &&~isempty(tt)
    veh_traj.Vehicle_ID = seg1.Vehicle_ID(1);
    veh_traj.t = [seg1.Global_Time(1),seg1.Global_Time(end)-seg1.Global_Time(1)];
    if seg1.Global_Time(1)<t_min
        disp('')
    end
    if veh_traj.t(2)/100+1~=length(x_bar)
        disp('')
    end
    veh_traj.x_traj = x_bar;
    veh_traj.y_traj = y_bar;
    veh_traj.v_traj = v_bar;
    if max(v_bar)>50 ||min(v_bar)<-1
        disp('')
    end
    veh_traj.acc_traj = acc_bar;
    veh_traj_set(counter)=veh_traj;
    counter = counter + 1;
end
    
for j=1:length(tt)
    t = seg1.Global_Time(1)+tt(j)*1000;
    if t>=t_min &&t<=t_max
    frame_idx = (t-t_min)/100+1;
    frames{frame_idx}=[frames{frame_idx};t vehicle_set(i) x_bar(j) y_bar(j) seg1.v_Class(1) seg1.v_length(1)*f2m seg1.v_Width(1)*f2m...
                                         v_bar(j) acc_bar(j) lane_ID(j)];
    end
end



end

for i=1:length(frames)
    frames{i} = array2table(frames{i},'VariableNames',{'Global_Time','Vehicle_ID','Local_X','Local_Y','v_Class','v_length'...
                             ,'v_Width','v_Vel','v_Acc','Lane_ID'});
end

%%
% idx = 53;
% figure(1)
% clf
% hold on
% for i=1:size(frames{idx},1)
%     draw_rec([frames{idx}.Local_X(i),frames{idx}.Local_Y(i)],[frames{idx}.v_Width(i) frames{idx}.v_length(i)],0,'b');
% end
% for i=1:length(lm)
%     plot([lm(i),lm(i)],[0,500],'r--')
% end
% axis equal
