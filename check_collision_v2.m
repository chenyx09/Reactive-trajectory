function TTC = check_collision_v2(affordance,traj,Ts1,T)
% afordance definition
% 1 Vehicle_ID(i) 
% 2 v_Vel(i) 
% 3 dis2cen 
% 4 fwd_dis 
% 5 fwd_vel 
% 6 rear_dis 
% 7 rear_vel 
% 8 left_front_Y 
% 9 left_front_X
% 10 left_front_vel 
% 11 left_rear_Y 
% 12 left_rear_X 
% 13 left_rear_vel 
% 14 right_front_Y 
% 15 right_front_X 
% 16 right_front_vel
% 17 right_rear_Y 
% 18 right_rear_X 
% 19 right_rear_vel
% 20 left_fwd_Y
% 21 left_fwd_vel
% 22 right_fwd_Y
% 23 right_fwd_vel
% 24 left_front_L
% 25 left_rear_L
% 26 right_front_L
% 27 right_rear_L
% 28 vehicle length

m = length(traj)/2;
if nargin<4
    T = (m-1)*Ts1;
end
L = affordance(28);
W = 2.4;
buffer_x = 0.1;
buffer_y = 0.1;
dis2cen = affordance(3);

tt = (0:m-1)*Ts1;
v0 = affordance(2);

fwd_vel         = v0 + affordance(5);
left_front_vel  = v0 + affordance(10);
left_rear_vel   = v0 + affordance(13);
right_front_vel = v0 + affordance(16);
right_rear_vel  = v0 + affordance(19);

left_front_X = affordance(9);
left_rear_X = affordance(12);
right_front_X = affordance(15);
right_rear_X = affordance(18);

left_front_L = affordance(24);
left_rear_L = affordance(25);
right_front_L = affordance(26);
right_rear_L = affordance(27);


if abs(fwd_vel-v0)>5
    fwd_vel = v0;
end
if abs(left_front_vel-v0)>15
    left_front_vel = v0;
end
if abs(left_rear_vel-v0)>15
    left_rear_vel = v0;
end

if abs(right_front_vel-v0)>15
    right_front_vel = v0;
end
if abs(right_rear_vel-v0)>15
    right_rear_vel = v0;
end




fwd_traj         = affordance(4)+fwd_vel*tt;
left_front_traj  = affordance(8)+left_front_vel*tt;
left_rear_traj   = affordance(6)+left_rear_vel*tt;
right_front_traj = affordance(14)+right_front_vel*tt;
right_rear_traj  = affordance(17)+right_rear_vel*tt;

% left_fwd_traj = affordance(20)+affordance(21)*tt;

% right_fwd_traj = affordance(22)+affordance(23)*tt;


y_traj = v0*tt + traj(1:m);
x_traj = traj(m+1:2*m)+dis2cen;

TTC = inf;
for i=1:m
    if (i-1)*Ts1>T
        break
    end
    if x_traj(i)-buffer_x<dis2cen-left_front_X && y_traj(i)>left_front_traj(i)-buffer_y && y_traj(i)-L< left_front_traj(i)+left_front_L+buffer_y %% left lane
        TTC = (i-1)*Ts1;
        break
    end
    if x_traj(i)-buffer_x<dis2cen-left_rear_X && y_traj(i)<left_rear_traj(i)+L+buffer_y && y_traj(i)>left_rear_traj(i)-left_rear_L-buffer_y
        TTC = (i-1)*Ts1;
        break
    end
    if abs(x_traj(i))<1.8-buffer_x+W/4 && y_traj(i)>fwd_traj(i)-buffer_y
        
        TTC = (i-1)*Ts1;
        break
        
    end
    if x_traj(i)+buffer_x>dis2cen+right_front_X && y_traj(i)>right_front_traj(i)-buffer_y && y_traj(i)-L< right_front_traj(i)+right_front_L+buffer_y
        
        TTC = (i-1)*Ts1;
        break
    end
    if x_traj(i)+buffer_x>dis2cen+right_rear_X && y_traj(i)<right_rear_traj(i)+L+buffer_y && y_traj(i)>right_rear_traj(i)-right_rear_L-buffer_y
        TTC = (i-1)*Ts1;
        break
    end
    if i>=2&&abs(x_traj(i)-x_traj(i-1))/abs(y_traj(i)-y_traj(i-1))>0.3
        TTC = 0;
        break
    end
end


if TTC<2
    
    figure(1)
    clf
    hold on
    draw_rec([x_traj(i),y_traj(i)-L/2],[W L],0,'b');
    draw_rec([0,fwd_traj(i)+2.5],[3.6 5],0,'r');
    draw_rec([dis2cen-left_front_X-W/2,left_front_traj(i)+2.5],[W 5],0,'r');
    draw_rec([dis2cen-left_rear_X-W/2,left_rear_traj(i)-2.5],[W 5],0,'r');

    
    draw_rec([dis2cen+right_front_X+W/2,right_front_traj(i)+2.5],[W 5],0,'r');
    draw_rec([dis2cen+right_rear_X+W/2,right_rear_traj(i)-2.5],[W 5],0,'r');

    plot([-1.8,-1.8],[y_traj(i)-7,y_traj(i)+7])
    plot([1.8,1.8],[y_traj(i)-7,y_traj(i)+7])
    axis equal
    if abs(x_traj(i))>2
        disp('')
    end
end

