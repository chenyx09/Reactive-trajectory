function TTC = check_collision(cur_affordance,prev_affordance,traj,delta_t,T)
%  frame.Vehicle_ID(i) frame.v_Vel(i) dis2cen fwd_dis rear_dis left_free_lb left_free_ub...
%                    left_free_fwd right_free_lb right_free_ub right_free_fwd
if nargin<4
    delta_t = 0.1;
end
m = length(traj)/2;
Ts1 = 0.5;
if nargin<5
    T = (m-1)*Ts1;
end




L = 5;
W = 2.6;
buffer_x = 0.1;
buffer_y = 0.3;
dis2cen = cur_affordance(3);
TTC = inf;
tt = (0:m-1)*Ts1;
v0 = cur_affordance(2);

fwd_spd         = v0 + (cur_affordance(4)-prev_affordance(4))/delta_t;
left_ahead_spd  = v0 + (cur_affordance(7)-prev_affordance(7))/delta_t;
left_rear_spd   = v0 + (cur_affordance(6)-prev_affordance(6))/delta_t;
left_fwd_spd    = v0 + (cur_affordance(8)-prev_affordance(8))/delta_t;

left_front_X = dis2cen-W/2-cur_affordance(9);
left_rear_X = dis2cen-W/2-cur_affordance(10);
right_ahead_spd = v0 + (cur_affordance(12)-prev_affordance(12))/delta_t;
right_rear_spd  = v0 + (cur_affordance(11)-prev_affordance(11))/delta_t;
right_fwd_spd   = v0 + (cur_affordance(13)-prev_affordance(13))/delta_t;
if abs(fwd_spd-v0)>5
    fwd_spd = v0;
end
if abs(left_ahead_spd-v0)>15
    left_ahead_spd = v0;
end
if abs(left_rear_spd-v0)>15
    left_rear_spd = v0;
end
if abs(left_fwd_spd-v0)>15
    left_fwd_spd = v0;
end
if abs(right_ahead_spd-v0)>15
    right_ahead_spd = v0;
end
if abs(right_rear_spd-v0)>15
    right_rear_spd = v0;
end
if abs(right_fwd_spd-v0)>15
    right_fwd_spd = v0;
end

right_front_X = dis2cen+W/2+cur_affordance(14);
right_rear_X = dis2cen+W/2+cur_affordance(15);

fwd_traj         = cur_affordance(4)+fwd_spd*tt;
left_ahead_traj  = cur_affordance(7)+left_ahead_spd*tt;
left_rear_traj   = cur_affordance(6)+left_rear_spd*tt;
left_fwd_traj    = cur_affordance(8)+left_fwd_spd*tt;
right_ahead_traj = cur_affordance(11)+right_ahead_spd*tt;
right_rear_traj  = cur_affordance(10)+right_rear_spd*tt;
right_fwd_traj   = cur_affordance(12)+right_fwd_spd*tt;


y_traj = v0*tt + traj(1:m);
x_traj = traj(m+1:2*m)+dis2cen;

for i=1:m
    if (i-1)*Ts1>T
        break
    end
    if x_traj(i)-W/2-buffer_x<left_front_X && y_traj(i)>left_ahead_traj(i)-buffer_y && y_traj(i)<left_fwd_traj(i)+L+buffer_y  %% left lane
        TTC = (i-1)*Ts1;
        break
    end
    if x_traj(i)-W/2-buffer_x<left_rear_X && y_traj(i)<left_rear_traj(i)+L+buffer_y
        TTC = (i-1)*Ts1;
        break
    end
    if abs(x_traj(i))<1.8-buffer_x+W/2 && y_traj(i)>fwd_traj(i)-buffer_y
        
        TTC = (i-1)*Ts1;
        break
        
    end
    if x_traj(i)+W/2+buffer_x>right_front_X && y_traj(i)>right_ahead_traj(i)-buffer_y && y_traj(i)<right_fwd_traj(i)+L+buffer_y
        
        TTC = (i-1)*Ts1;
        break
    end
    if x_traj(i)+W/2+buffer_x>right_rear_X && y_traj(i)<right_rear_traj(i)+L+buffer_y
        TTC = (i-1)*Ts1;
        break
    end
end


% if TTC<2
%     
%     figure(1)
%     clf
%     hold on
%     draw_rec([x_traj(i),y_traj(i)-L/2],[W L],0,'b');
%     draw_rec([0,fwd_traj(i)+2.5],[3.6 5],0,'r');
%     draw_rec([left_front_X-W/2,left_ahead_traj(i)+2.5],[W 5],0,'r');
%     draw_rec([left_rear_X-W/2,left_rear_traj(i)-2.5],[W 5],0,'r');
%     draw_rec([-3.6,left_fwd_traj(i)+2.5],[3.6 5],0,'r');
%     
%     draw_rec([right_front_X+W/2,right_ahead_traj(i)+2.5],[W 5],0,'r');
%     draw_rec([right_rear_X+W/2,right_rear_traj(i)-2.5],[W 5],0,'r');
%     draw_rec([3.6,right_fwd_traj(i)+2.5],[3.6 5],0,'r');
%     plot([-1.8,-1.8],[y_traj(i)-7,y_traj(i)+7])
%     plot([1.8,1.8],[y_traj(i)-7,y_traj(i)+7])
%     axis equal
%     if abs(x_traj(i))>2
%         disp('')
%     end
% end

