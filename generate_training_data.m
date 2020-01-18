%% generate training data
if exist('data.mat','file')==2&&0
    load data
    i=1;
    d = zeros(M,1);
    while i<=size(positive_data,1)
        i
        traj1=positive_data(i,end-2*m:end-1);
        for j=1:M
            d(j)=scaled_inf_norm(traj1,traj_base_kept(j,:));
        end
        [min_d,idx]=min(d);
        if min_d<=delta
            positive_data(i,end) = idx;
            i=i+1;
        else
            [cover_set,cover_score] = double_traj_cover(traj_base_kept,traj1,delta);
            [min_score,idx]=min(cover_score);
            if min_score<delta
                positive_data(i,:) = [positive_data(i,1:end-2*m-1) traj_base_kept(cover_set(idx,1),:) cover_set(idx,1)];
                positive_data = [positive_data;positive_data(i,1:end-2*m-1) traj_base_kept(cover_set(idx,2),:) cover_set(idx,2)];
                i=i+1;
            else
                unclassified_set = positive_data(i,1:end-1);
                positive_data(i,:)=[];
            end
        end
    end
else
%     generate_traj_bases;
    opts = detectImportOptions('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-1.csv');
    TT = 3;
    Ts = 0.1;
    Ts1 = 0.5;
    t_traj = 0:Ts1:TT;
    positive_data=[];
    traj_pool=[];
    unclassified_set=[];
    x_diff = [];
    cc =0;
    for part=1:11
        
        data=readtable(['Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-',num2str(part),'.csv'],opts);
        data = data(contains(data.Location,'us-101'),:);
        if ~isempty(data)
            data = sortrows(data,'Global_Time');
            
            if ~exist('Bez_matr','var')
                bezier_regressor;
            end
            
            [a,b]=hist(data.Global_Time,unique(data.Global_Time));
            c=sortrows([a' b],1,'descend');
            T_min = min(b);
            T_max = max(b);
            processed_frames=[];
            affordance_set = {};
            counter = 0;
            
            M = size(traj_base,1);
            d = zeros(M,1);
            
            for n=1:size(c,1)
                
                if c(n,1)<20
                    break
                end
                t0 = c(n,2);
                if n<=5||min(abs(t0-c(1:n-1,2)))>8000
                    %         n
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
                            if max(dp.x_traj)-min(dp.x_traj)>3
                                disp('')
                            end
                            x_diff=[x_diff;max(dp.x_traj)-min(dp.x_traj)];
                            
                            dp.affordance1=table2array(affordance1(affordance1.Vehicle_ID==veh_traj_set(i).Vehicle_ID,:));
                            dp.affordance2=table2array(affordance2(affordance2.Vehicle_ID==veh_traj_set(i).Vehicle_ID,:));
                            dp.affordance3=table2array(affordance3(affordance3.Vehicle_ID==veh_traj_set(i).Vehicle_ID,:));
                            %             training_set=[training_set;dp];
                            if ~(isempty(dp.affordance1)||isempty(dp.affordance2)||isempty(dp.affordance3))
                                
                                y_nom = veh_traj_set(i).v_traj(1)*(0:Ts1:TT);
                                delta_y = dp.y_traj - y_nom;
                                
                                traj1 = [delta_y dp.x_traj];
                                for j=1:M
                                    d(j)=scaled_inf_norm(traj1,traj_base(j,:));
                                end
                                [min_d,idx]=min(d);
                                if min_d<=delta
                                    positive_data = [positive_data;dp.affordance1 dp.affordance2 dp.affordance3 traj1 idx];
                                    cc=cc+1
                                else
                                    [cover_set,cover_score] = double_traj_cover(traj_base,traj1,delta);
                                    [min_score,idx]=min(cover_score);
                                    if min_score<1.3*delta
                                        positive_data = [positive_data;dp.affordance1 dp.affordance2 dp.affordance3 traj_base(cover_set(idx,1),:) cover_set(idx,1)];
                                        positive_data = [positive_data;dp.affordance1 dp.affordance2 dp.affordance3 traj_base(cover_set(idx,2),:) cover_set(idx,2)];
                                        cc = cc+2
                                    else
                                        %                 traj_pool = [traj_pool;traj1];
                                        unclassified_set = [unclassified_set;dp.affordance1 dp.affordance2 dp.affordance3 traj1];
                                    end
                                    %                 M = M+1
                                    %                 training_data = [training_data;dp.affordance1 dp.affordance2 dp.affordance3 M];
                                end
                            end
                        end
                        %         dp.x_traj = interp
                    end
                end
            end
            
            
            delta = 0.5;
            if size(unclassified_set,1)>1000
                [traj_base_kept,N_cover]=traj_sparsification(unclassified_set(:,end-2*m+1:end),m,delta);
                traj_base = [traj_base;traj_base_kept(N_cover>5,:)];
                i=1;
                while i<=size(unclassified_set)
                    for j=1:size(traj_base,1)
                        d(j)=scaled_inf_norm(unclassified_set(i,end-2*m+1:end),traj_base(j,:));
                    end
                    [min_d,idx]=min(d);
                    if min_d<=delta
                        positive_data = [positive_data;unclassified_set(i,:)  idx];
                        cc=cc+1
                        unclassified_set(i,:)=[];
                    else
                        
                        [cover_set,cover_score] = double_traj_cover(traj_base,unclassified_set(i,end-2*m+1:end),delta);
                        [min_score,idx]=min(cover_score);
                        if min_score<1.3*delta
                            positive_data = [positive_data;unclassified_set(i,1:end-2*m+1) traj_base(cover_set(idx,1),:) cover_set(idx,1)];
                            positive_data = [positive_data;unclassified_set(i,1:end-2*m+1) traj_base(cover_set(idx,2),:) cover_set(idx,2)];
                            cc=cc+2
                        else
                            i=i+1;
                        end
                    end
                end
                
                
            end
            M = size(traj_base,1)
            size(unclassified_set)
        end
    end
    %% get rid of the underrepresented bases
    threshold = 10;
    [a,b] = hist(positive_data(:,end),1:M);
    under_rep_base = find(a<threshold);
    kept_base = find(a>=threshold);
    traj_base_kept = traj_base(kept_base,:);
    d = zeros(length(kept_base),1);
    i=1;
    while i<=size(positive_data,1)
        idx = positive_data(i,end);
        if ismember(idx,kept_base)
            positive_data(i,end) = find(kept_base==idx);
            i=i+1;
        else
            
            for j=1:size(traj_base_kept,1)
                d(j)=scaled_inf_norm(positive_data(i,end-2*m:end-1),traj_base_kept(j,:));
            end
            [min_d,min_idx]=min(d);
            if min_d<=delta
                positive_data(i,end) = min_idx;
                i=i+1;
            else
                [cover_set,cover_score] = double_traj_cover(traj_base_kept,positive_data(i,end-2*m:end-1),delta);
                [min_score,min_idx]=min(cover_score);
                if min_score<1.3*delta
                    positive_data(i,end-2*m+1:end) = [traj_base_kept(cover_set(min_idx,1),:) cover_set(min_idx,1)];
                    positive_data = [positive_data;positive_data(i,1:end-2*m-1) traj_base_kept(cover_set(min_idx,2),:) cover_set(min_idx,2)];
                    i=i+1;
                else
                    %                 traj_pool = [traj_pool;traj1];
                    positive_data(i,:)=[];
                    unclassified_set = [unclassified_set;positive_data(i,1:end-1)];
                end
            end
        end
        
    end
    %% handle the unclassified set
    i=1;
    while i<=size(unclassified_set)
        for j=1:size(traj_base_kept,1)
            d(j)=scaled_inf_norm(unclassified_set(i,end-2*m+1:end),traj_base_kept(j,:));
        end
        [min_d,idx]=min(d);
        if min_d<=delta
            positive_data = [positive_data;unclassified_set(i,1:end) idx];
            unclassified_set(i,:)=[];
        else
            
            [cover_set,cover_score] = double_traj_cover(traj_base_kept,unclassified_set(i,end-2*m+1:end),delta);
            [min_score,idx]=min(cover_score);
            if min_score<delta
                positive_data = [positive_data;unclassified_set(i,1:end) cover_set(idx,1)];
                positive_data = [positive_data;unclassified_set(i,1:end) cover_set(idx,2)];
            else
                traj_base_kept = [traj_base_kept;unclassified_set(i,end-2*m+1:end)];
                positive_data = [positive_data;unclassified_set(i,1:end) size(traj_base_kept,1)];
                unclassified_set(i,:)=[];
            end
        end
    end
    
end

%% rule out conflicting positive data
M = size(traj_base_kept,1);
[a,b] = hist(positive_data(:,end),1:M);
% N_n = round(size(positive_data,1)*0.2);
negative_data = [];
i=1;
counter = 0;
while i<=size(positive_data,1)
    TTC = check_collision(positive_data(i,31:45),positive_data(i,1:15),traj_base_kept(positive_data(i,end),:),2*Ts,2);
    if TTC<inf
        positive_data(i,:)=[];
        counter= counter+1
    else
        i=i+1;
    end
end

for i=1:M
    positive_data_cell{i}=positive_data(positive_data(:,end)==i,:);
end
%% generate negative data
N_p = size(positive_data,1);
ideal_size = max(10000,a*0.8);
idx = randsample(1:N_p,N_p);
for i=1:M
    negative_data_cell{i}=[];
end
for i=1:length(idx)
    i
    for j=1:M
        if size(negative_data_cell{j},1)<3*ideal_size(j)
            TTC = check_collision(positive_data(idx(i),31:45),positive_data(idx(i),1:15),traj_base_kept(j,:),2*Ts,2);
            if TTC<inf
                negative_data_cell{j} = [negative_data_cell{j}; positive_data(idx(i),1:45) TTC];
            end
        end
    end
end
for i=1:M
    negative_data_cell{i} = sortrows(negative_data_cell{i},46,'ascend');
end
% for i=1:M
%     if size(negative_data_cell{i},1)>ideal_size(i)
%         negative_data_cell{i}=sortrows(negative_data_cell{i},46,'ascend');
%         negative_data_cell{i} = negative_data_cell{i}(1:ideal_size(i),:);
%     end
% end


%% Nonlinear features
%   'Vehicle_ID','v_Vel','dis2cen','fwd_dis','rear_dis','left_free_lb','left_free_ub','left_free_fwd',...
%   'left_dis_front','left_dis_rear','right_free_lb','right_free_ub','right_free_fwd','right_dis_front','right_dis_rear'
N_p = size(positive_data,1);
h = mss_asd(12,2);

Fh = size(h,1);
F = Fh+33;
x_norm = max(abs(positive_data(:,[2,3,4,6,7,9,10,11,12,14,15])));
for i=1:M
    i
    phi_positive{i} = [];
    phi_negative{i} = [];
    xx=[positive_data_cell{i}(:,30+[2,3,4,6,7,9,10,11,12,14,15]) ones(size(positive_data_cell{i},1),1)]./[x_norm 1];
    for j=1:Fh
        counter=1;
        vec=ones(size(positive_data_cell{i},1),3);
        for k=1:size(h,2)
            if h(j,k)==1
                vec(:,counter)=xx(:,k);
                counter = counter+1;
            elseif h(j,k)==2
                vec(:,counter)=xx(:,k).^2;
                counter = counter+1;
            elseif h(j,k)==2
                vec(:,counter)=xx(:,k).^3;
                counter = counter+1;
            end
        end
        phi_positive{i}(:,j)=vec(:,1).*vec(:,2).*vec(:,3);
    end
    phi_positive{i}=[phi_positive{i} tanh(3*positive_data_cell{i}(:,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm) ...
        exp(-(positive_data_cell{i}(:,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm).^2)  positive_data_cell{i}(:,[2,3,4,6,7,9,10,11,12,14,15])./x_norm];
    
    xx=[negative_data_cell{i}(:,30+[2,3,4,6,7,9,10,11,12,14,15]) ones(size(negative_data_cell{i},1),1)]./[x_norm 1];
    for j=1:Fh
        counter=1;
        vec=ones(size(negative_data_cell{i},1),3);
        for k=1:size(h,2)
            if h(j,k)==1
                vec(:,counter)=xx(:,k);
                counter = counter+1;
            elseif h(j,k)==2
                vec(:,counter)=xx(:,k).^2;
                counter = counter+1;
            elseif h(j,k)==2
                vec(:,counter)=xx(:,k).^3;
                counter = counter+1;
            end
        end
        phi_negative{i}(:,j)=vec(:,1).*vec(:,2).*vec(:,3);
    end
    phi_negative{i}=[phi_negative{i} tanh(3*negative_data_cell{i}(:,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm) ...
        exp(-(negative_data_cell{i}(:,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm).^2)  negative_data_cell{i}(:,[2,3,4,6,7,9,10,11,12,14,15])./x_norm];

%     for j=1:size(positive_data_cell{i},1)
%         xx=[positive_data_cell{i}(j,30+[2,3,4,6,7,9,10,11,12,14,15]) 1]./[x_norm 1];
%         entry=zeros(1,Fh);
%         for k=1:size(h,1)
%             entry(k)=prod(xx.^h(k,:));
%         end
% %         entry(k+1:k+11)=positive_data_cell{i}(j,[2,3,4,6,7,9,10,11,12,14,15])./x_norm;
%         phi_positive{i}=[phi_positive{i};[entry tanh(3*positive_data_cell{i}(j,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm)...
%           exp(-(positive_data_cell{i}(j,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm).^2)  positive_data_cell{i}(j,[2,3,4,6,7,9,10,11,12,14,15])./x_norm]];
%     end
%     for j=1:size(negative_data_cell{i},1)
%         xx=[negative_data_cell{i}(j,30+[2,3,4,6,7,9,10,11,12,14,15]) 1]./[x_norm 1];
%         entry=zeros(1,Fh);
%         for k=1:size(h,1)
%             entry(k)=prod(xx.^h(k,:));
%         end
% %         entry(k+1:k+11)=negative_data_cell{i}(j,[2,3,4,6,7,9,10,11,12,14,15])./x_norm;
%         phi_negative{i}=[phi_negative{i};[entry tanh(3*negative_data_cell{i}(j,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm)...
%             exp(-(negative_data_cell{i}(j,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm).^2) negative_data_cell{i}(j,[2,3,4,6,7,9,10,11,12,14,15])./x_norm]];
%     end
end


%%



%% draw example trajectory
% idx = [315,4218,1033,877,3767];
% figure(1)
% clf
% hold on
% % v0 = traj_pool(idx,2*m+1);
% for n = 1:length(idx)
%
% v0 = 15;
% y_traj = traj_pool(idx(n),1:m)+v0*(0:Ts1:TT);
% x_traj = traj_pool(idx(n),m+1:2*m);
%
% for i = 1:m
%
%     ellipse(sqrt(delta)/y_scaling(i),sqrt(delta)/x_scaling(i),0,y_traj(i),x_traj(i),'r');
% end
%
% axis equal
%
% end
