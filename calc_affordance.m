function veh_affordance = calc_affordance(frame)

veh_affordance=[];
M = 50;
obs = calc_obs(frame);
min_free_space = 6;
for i=1:size(frame,1)
    lane_ID = frame.Lane_ID(i);
    X = frame.Local_X(i);
    Y = frame.Local_Y(i);
    L = frame.v_length(i);
    W = frame.v_Width(i);
    dis2cen = X-(lane_ID*3.6-1.8);
    if X<=-2.5 ||X>=21.5
        break
    end
    if abs(dis2cen)>2
        warning('wrong lane ID')
    end
    if frame.v_Vel<0
        disp('')
    end
if frame.Vehicle_ID(i)==555
    disp('')
end
    switch lane_ID
        case 1
            left_free_ub = 0;
            left_free_lb = 0;
            left_free_fwd = M;
            left_dis_front = dis2cen-W/2+1.8;
            left_dis_rear  = dis2cen-W/2+1.8;
            obs1 = obs{lane_ID}(obs{lane_ID}(:,2)>Y,:);
            obs2 = obs{lane_ID}(obs{lane_ID}(:,2)<Y,:);
            if ~isempty(obs1)
                fwd_dis = obs1(1,1)-Y;
            else
                fwd_dis = M;
            end
            if ~isempty(obs2)
                rear_dis = Y-L-obs2(end,2);
            else
                rear_dis = M;
            end
            obs1 = obs{lane_ID+1}(obs{lane_ID+1}(:,2)>Y,:);
            obs2 = obs{lane_ID+1}(obs{lane_ID+1}(:,2)<Y,:);
            right_dis_front = 3.6;
            right_dis_rear  = 3.6;
            if ~isempty(obs1)
                right_dis_front = min(right_dis_front,obs1(1,3)-X-W/2);
                if obs1(1,1)<Y
                    right_free_ub = 0;
                    right_free_lb = 0;
                else
                    right_free_ub = obs1(1,1)-Y;
                    if ~isempty(obs2)
                        right_free_lb = Y-obs2(end,2);
                    else
                        right_free_lb = M;
                    end
                end
                
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
                right_free_fwd = obs1(j,2)-Y;
     
            else
                right_free_ub = M;
                right_free_fwd = 0;
                if ~isempty(obs2)
                    right_free_lb = Y-obs2(end,2);
                else
                    right_free_lb = M;
                end
            end
            if ~isempty(obs2)
                right_dis_rear = min(right_dis_rear,obs2(end,3)-X-W/2);
            end
        case {2,3,4,5}
            obs1 = obs{lane_ID}(obs{lane_ID}(:,2)>Y,:);
            obs2 = obs{lane_ID}(obs{lane_ID}(:,2)<Y,:);
            if ~isempty(obs1)
                fwd_dis = obs1(1,1)-Y;
            else
                fwd_dis = M;
            end
            if ~isempty(obs2)
                rear_dis = Y-L-obs2(end,2);
            else
                rear_dis = M;
            end
            obs1 = obs{lane_ID+1}(obs{lane_ID+1}(:,2)>Y,:);
            obs2 = obs{lane_ID+1}(obs{lane_ID+1}(:,2)<Y,:);
            right_dis_front = 3.6;
            right_dis_rear  = 3.6;
            if ~isempty(obs1)
                    right_dis_front = min(right_dis_front,obs1(1,3)-X-W/2);
                if obs1(1,1)<Y
                    right_free_ub = 0;
                    right_free_lb = 0;
                else
                    right_free_ub = obs1(1,1)-Y;
                    if ~isempty(obs2)
                        right_free_lb = Y-obs2(end,2);
                    else
                        right_free_lb = M;
                    end
                end
                
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
                right_free_fwd = obs1(j,2)-Y;
     
            else
                right_free_ub = M;
                right_free_fwd = 0;
                if ~isempty(obs2)
                    right_free_lb = Y-obs2(end,2);
                else
                    right_free_lb = M;
                end
            end
            if ~isempty(obs2)
                    right_dis_rear = min(right_dis_rear,obs2(end,3)-X-W/2);
            end
            
            obs1 = obs{lane_ID-1}(obs{lane_ID-1}(:,2)>Y,:);
            obs2 = obs{lane_ID-1}(obs{lane_ID-1}(:,2)<Y,:);
            left_dis_front = 3.6;
            left_dis_rear  = 3.6;
            if ~isempty(obs1)
                    left_dis_front = min(left_dis_front,X-obs1(1,3)-W/2);
                if obs1(1,1)<Y
                    left_free_ub = 0;
                    left_free_lb = 0;
                else
                    left_free_ub = obs1(1,1)-Y;
                    if ~isempty(obs2)
                        left_free_lb = Y-obs2(end,2);
                    else
                        left_free_lb = M;
                    end
                end
                
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
                left_free_fwd = obs1(j,2)-Y;
     
            else
                left_free_ub = M;
                left_free_fwd = 0;
                if ~isempty(obs2)
                    left_free_lb = Y-obs2(end,2);
                else
                    left_free_lb = M;
                end
            end
            if ~isempty(obs2)
                    left_dis_rear = min(left_dis_rear,X-obs2(end,3)-W/2);
            end
        case 6
            obs1 = obs{lane_ID}(obs{lane_ID}(:,2)>Y,:);
            obs2 = obs{lane_ID}(obs{lane_ID}(:,2)<Y,:);
            right_dis_front = 1.8-dis2cen-W/2;
            right_dis_rear  = 1.8-dis2cen-W/2;
            if ~isempty(obs1)
                fwd_dis = obs1(1,1)-Y;
            else
                fwd_dis = M;
            end
            if ~isempty(obs2)
                rear_dis = Y-L-obs2(end,2);
            else
                rear_dis = M;
            end
            right_free_ub = 0;
            right_free_lb = 0;
            right_free_fwd = M;
            
            obs1 = obs{lane_ID-1}(obs{lane_ID-1}(:,2)>Y,:);
            obs2 = obs{lane_ID-1}(obs{lane_ID-1}(:,2)<Y,:);
            left_dis_front = 3.6;
            left_dis_rear  = 3.6;
            if ~isempty(obs1)

                    left_dis_front = min(left_dis_front,X-obs1(1,3)-W/2);

                if obs1(1,1)<Y
                    left_free_ub = 0;
                    left_free_lb = 0;
                else
                    left_free_ub = obs1(1,1)-Y;
                    if ~isempty(obs2)
                        left_free_lb = Y-obs2(end,2);
                    else
                        left_free_lb = M;
                    end
                end
                
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
                left_free_fwd = obs1(j,2)-Y;
     
            else
                left_free_ub = M;
                left_free_fwd = 0;
                if ~isempty(obs2)
                    left_free_lb = Y-obs2(end,2);
                else
                    left_free_lb = M;
                end
            end
            if ~isempty(obs2)
                    left_dis_rear = min(left_dis_rear,X-obs2(end,3)-W/2);
            end
    end

    veh_affordance = [veh_affordance;frame.Vehicle_ID(i) frame.v_Vel(i) dis2cen fwd_dis rear_dis left_free_lb left_free_ub...
                      left_free_fwd left_dis_front left_dis_rear right_free_lb right_free_ub right_free_fwd right_dis_front right_dis_rear];
end
veh_affordance(:,4:end)=max(0,veh_affordance(:,4:end));
veh_affordance(:,4:end)=min(M,veh_affordance(:,4:end));
veh_affordance = array2table(veh_affordance,'VariableNames',{'Vehicle_ID','v_Vel','dis2cen','fwd_dis','rear_dis','left_free_lb',...
                             'left_free_ub','left_free_fwd','left_dis_front','left_dis_rear','right_free_lb','right_free_ub',...
                             'right_free_fwd','right_dis_front','right_dis_rear'});
% figure(1)
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

            
        