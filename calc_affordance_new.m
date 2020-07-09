function veh_affordance = calc_affordance_new(frame)

veh_affordance=[];
M = 50;
obs = calc_obs(frame);

for i=1:size(frame,1)
    lane_ID = frame.Lane_ID(i);
    X = frame.Local_X(i);
    Y = frame.Local_Y(i);
    L = frame.v_length(i);
    W = frame.v_Width(i);
    v0 = frame.v_Vel(i);
    dis2cen = X-(lane_ID*3.6-1.8);
    min_free_space = max(2*L,6);
%     if frame.Vehicle_ID(i)==1098
%         disp('')
%     end
    if X>=-2.5 &&X<=21.5

    if abs(dis2cen)>2
        warning('wrong lane ID')
    end
    if frame.v_Vel<0
        disp('')
    end
    
    switch lane_ID
        case 1
            left_front_Y = 0;
            left_rear_Y = 0;
            left_front_vel = 0;
            left_rear_vel = 0;
            left_front_X = dis2cen-W/2+1.8;
            left_rear_X  = dis2cen-W/2+1.8;
            left_fwd_Y = M;
            left_fwd_vel = 0;
            left_front_L = 0;
            left_rear_L = 0;
            if ~isempty(obs{lane_ID})
                obs1 = obs{lane_ID}(obs{lane_ID}(:,2)>Y,:);
                obs2 = obs{lane_ID}(obs{lane_ID}(:,2)<Y,:);
            else
                obs1=[];
                obs2=[];
            end
            if ~isempty(obs1)
                fwd_dis = obs1(1,1)-Y;
                fwd_vel = obs1(1,5)-v0;
            else
                fwd_dis = M;
                fwd_vel = 0;
            end
            
            if ~isempty(obs2)
                rear_dis = Y-L-obs2(end,2);
                rear_vel = obs2(end,5)-v0;
            else
                rear_dis = M;
                rear_vel = 0;
            end
            if ~isempty(obs{lane_ID+1})
                obs1 = obs{lane_ID+1}(obs{lane_ID+1}(:,1)>Y-L/2,:);
                obs2 = obs{lane_ID+1}(obs{lane_ID+1}(:,1)<Y-L/2,:);
            else
                obs1=[];
                obs2=[];
            end
            right_front_X = 3.6;
            
            if ~isempty(obs1)
                right_front_X = min(right_front_X,obs1(1,3)-X-W/2);
                right_front_vel = obs1(1,5)-v0;
                right_front_Y = obs1(1,1)-Y;
                right_front_L = obs1(1,2)-obs1(1,1);
                j=1;
                while j<=size(obs1,1)-1
                    if obs1(j+1,1)-obs1(j,2)>min_free_space
                        break
                    else
                        j=j+1;
                    end
                    if obs1(j,2)-Y>M
                        break
                    end
                end
                right_fwd_Y = obs1(j,2)-Y-L;
                right_fwd_vel = obs1(j,5)-v0;
            else
                right_front_Y = M;
                right_front_vel = 0;
                right_fwd_Y = -M;
                right_fwd_vel = 0;
                right_front_L = 0;
            end
            right_rear_X  = 3.6;
            if ~isempty(obs2)
                right_rear_X = min(right_rear_X,obs2(end,3)-X-W/2);
                right_rear_Y = Y-obs2(end,2);
                right_rear_vel = obs2(end,5)-v0;
                right_rear_L = obs2(end,2)-obs2(end,1);
            else
                right_rear_Y = M;
                right_rear_vel = 0;
                right_rear_L = 0;
            end
        case {2,3,4,5}
            if ~isempty(obs{lane_ID})
                obs1 = obs{lane_ID}(obs{lane_ID}(:,2)>Y,:);
                obs2 = obs{lane_ID}(obs{lane_ID}(:,2)<Y,:);
            else    
                obs1=[];
                obs2=[];
            end
            if ~isempty(obs1)
                fwd_dis = obs1(1,1)-Y;
                fwd_vel = obs1(1,5)-v0;
            else
                fwd_dis = M;
                fwd_vel = 0;
            end
            
            if ~isempty(obs2)
                rear_dis = Y-L-obs2(end,2);
                rear_vel = obs2(end,5)-v0;
            else
                rear_dis = M;
                rear_vel = 0;
            end
            if ~isempty(obs{lane_ID+1})
            obs1 = obs{lane_ID+1}(obs{lane_ID+1}(:,1)>Y-L/2,:);
            obs2 = obs{lane_ID+1}(obs{lane_ID+1}(:,1)<Y-L/2,:);
            else
                obs1=[];
                obs2=[];
            end
            right_front_X = 3.6;
            
            if ~isempty(obs1)
                right_front_X = min(right_front_X,obs1(1,3)-X-W/2);
                right_front_vel = obs1(1,5)-v0;
                right_front_Y = obs1(1,1)-Y;
                right_front_L = obs1(1,2)-obs1(1,1);
                j=1;
                while j<=size(obs1,1)-1
                    if obs1(j+1,1)-obs1(j,2)>min_free_space
                        break
                    else
                        j=j+1;
                    end
                    if obs1(j,2)-Y>M
                        break
                    end
                end
                right_fwd_Y = obs1(j,2)-Y-L;
                right_fwd_vel = obs1(j,5)-v0;
            else
                right_front_Y = M;
                right_front_vel = 0;
                right_fwd_Y = -M;
                right_fwd_vel = 0;
                right_front_L = 0;
            end
            right_rear_X  = 3.6;
            if ~isempty(obs2)
                right_rear_X = min(right_rear_X,obs2(end,3)-X-W/2);
                right_rear_Y = Y-obs2(end,2);
                right_rear_vel = obs2(end,5)-v0;
                right_rear_L = obs2(end,2)-obs2(end,1);
            else
                right_rear_Y = M;
                right_rear_vel = 0;
                right_rear_L = 0;
            end
            
            if ~isempty(obs{lane_ID-1})
                obs1 = obs{lane_ID-1}(obs{lane_ID-1}(:,1)>Y-L/2,:);
                obs2 = obs{lane_ID-1}(obs{lane_ID-1}(:,1)<Y-L/2,:);
            else
                obs1=[];
                obs2=[];
            end
            left_front_X = 3.6;
            
            if ~isempty(obs1)
                left_front_X = min(left_front_X,X-obs1(1,4)-W/2);
                left_front_vel = obs1(1,5)-v0;
                left_front_Y = obs1(1,1)-Y;
                left_front_L = obs1(1,2)-obs1(1,1);
                j=1;
                while j<=size(obs1,1)-1
                    if obs1(j+1,1)-obs1(j,2)>min_free_space
                        break
                    else
                        j=j+1;
                    end
                    if obs1(j,2)-Y>M
                        break
                    end
                end
                left_fwd_Y = obs1(j,2)-Y-L;
                left_fwd_vel = obs1(j,5)-v0;
            else
                left_front_Y = M;
                left_front_vel = 0;
                left_fwd_Y = -M;
                left_fwd_vel = 0;
                left_front_L = 0;
            end
            left_rear_X  = 3.6;
            if ~isempty(obs2)
                left_rear_X = min(left_rear_X,X-obs2(end,4)-W/2);
                left_rear_Y = Y-obs2(end,2);
                left_rear_vel = obs2(end,5)-v0;
                left_rear_L = obs2(end,2)-obs2(end,1);
            else
                left_rear_Y = M;
                left_rear_vel = 0;
                left_rear_L = 0;
            end
        case 6
            right_front_Y = 0;
            right_rear_Y = 0;
            right_front_X = 1.8-dis2cen-W/2;
            right_rear_X  = 1.8-dis2cen-W/2;
            right_front_vel = 0;
            right_rear_vel = 0;
            right_fwd_Y = M;
            right_fwd_vel = 0;
            right_front_L = 0;
            right_rear_L = 0;
            if ~isempty(obs{lane_ID})
                obs1 = obs{lane_ID}(obs{lane_ID}(:,2)>Y,:);
                obs2 = obs{lane_ID}(obs{lane_ID}(:,2)<Y,:);
            else
                obs1=[];
                obs2=[];
            end
            
            if ~isempty(obs1)
                fwd_dis = obs1(1,1)-Y;
                fwd_vel = obs1(1,5)-v0;
            else
                fwd_dis = M;
                fwd_vel = 0;
            end
            if ~isempty(obs2)
                rear_dis = Y-L-obs2(end,2);
                rear_vel = obs2(end,5)-v0;
            else
                rear_dis = M;
                rear_vel = 0;
            end
            
            if ~isempty(obs{lane_ID-1})
                obs1 = obs{lane_ID-1}(obs{lane_ID-1}(:,1)>Y-L/2,:);
                obs2 = obs{lane_ID-1}(obs{lane_ID-1}(:,1)<Y-L/2,:);
            else
                obs1 = [];
                obs2 = [];
            end
            left_front_X = 3.6;
            
            if ~isempty(obs1)
                left_front_X = min(left_front_X,X-obs1(1,4)-W/2);
                left_front_vel = obs1(1,5)-v0;
                left_front_Y = obs1(1,1)-Y;
                left_front_L = obs1(1,2)-obs1(1,1);
                j=1;
                while j<=size(obs1,1)-1
                    if obs1(j+1,1)-obs1(j,2)>min_free_space
                        break
                    else
                        j=j+1;
                    end
                    if obs1(j,2)-Y>M
                        break
                    end
                end
                left_fwd_Y = obs1(j,2)-Y-L;
                left_fwd_vel = obs1(j,5)-v0;
            else
                left_front_Y = M;
                left_front_vel = 0;
                left_fwd_Y = -M;
                left_fwd_vel = 0;
                left_front_L = 0;
            end
            left_rear_X  = 3.6;
            if ~isempty(obs2)
                left_rear_X = min(left_rear_X,X-obs2(end,4)-W/2);
                left_rear_Y = Y-obs2(end,2);
                left_rear_vel = obs2(end,5)-v0;
                left_rear_L = obs2(end,2)-obs2(end,1);
            else
                left_rear_Y = M;
                left_rear_vel = 0;
                left_rear_L = 0;
            end
    end
    
    veh_affordance = [veh_affordance;frame.Vehicle_ID(i) frame.v_Vel(i) dis2cen fwd_dis fwd_vel rear_dis rear_vel left_front_Y left_front_X left_front_vel left_rear_Y...
         left_rear_X left_rear_vel right_front_Y right_front_X right_front_vel right_rear_Y right_rear_X right_rear_vel left_fwd_Y left_fwd_vel right_fwd_Y right_fwd_vel,left_front_L,left_rear_L,right_front_L,right_rear_L L];
    end
end
% veh_affordance(:,4:end)=max(0,veh_affordance(:,4:end));
veh_affordance(:,[4,6,8,11,14,17,20,22])=min(M,veh_affordance(:,[4,6,8,11,14,17,20,22]));
veh_affordance = array2table(veh_affordance,'VariableNames',{'Vehicle_ID', 'v_Vel', 'dis2cen', 'fwd_dis', 'fwd_vel', 'rear_dis', 'rear_vel', 'left_front_Y','left_front_X','left_front_vel', 'left_rear_Y',...
     'left_rear_X', 'left_rear_vel', 'right_front_Y', 'right_front_X', 'right_front_vel', 'right_rear_Y', 'right_rear_X', 'right_rear_vel','left_fwd_Y','left_fwd_vel','right_fwd_Y','right_fwd_vel','left_front_L','left_rear_L','right_front_L','right_rear_L','L'});
% figure(1)
% lm = linspace(0,21.6,7);
% clf
% hold on
% for i=1:size(frame,1)
%     draw_rec([frame.Local_X(i),frame.Local_Y(i)-frame.v_length(i)/2],[frame.v_Width(i) frame.v_length(i)],0,'b');
% 
%     text(frame.Local_X(i)+frame.v_Width(i)/2,frame.Local_Y(i)-frame.v_length(i)/2,num2str(frame.Vehicle_ID(i)),'HorizontalAlignment','right')
% end
% for i=1:length(lm)
%     plot([lm(i),lm(i)],[0,500],'r--')
% end
% axis equal


