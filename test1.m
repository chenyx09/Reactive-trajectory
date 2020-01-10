
i=1;
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
        if min_score<1.3*delta
            positive_data(i,:) = [positive_data(i,1:end-2*m-1) traj_base_kept(cover_set(idx,1),:) cover_set(idx,1)];
            positive_data = [positive_data;positive_data(i,1:end-2*m-1) traj_base_kept(cover_set(idx,2),:) cover_set(idx,2)];
            i=i+1;
        else
            unclassified_set = positive_data(i,1:end-1);
            positive_data(i,:)=[];
        end
    end
end