function [traj_base,N_cover]=traj_sparsification(traj_pool,m,delta)
N = size(traj_pool,1);
adj = zeros(N,N);

for i=1:N
    traj1 = traj_pool(i,1:2*m);
    for j=1:i
        
        traj2 = traj_pool(j,1:2*m);
        adj(i,j)=scaled_inf_norm(traj1,traj2);
        adj(j,i)=adj(i,j);
    end
end

%% sparsification

adj_binary = adj<=delta;
picked_idx=[];
remain_idx = 1:N;
remain_matr = adj_binary;
N_cover=[];
while 1
    cover = sum(remain_matr);
    [n_cover,idx] = max(cover);
    picked_idx = [picked_idx remain_idx(idx)];
    covered_idx = find(remain_matr(idx,:));
    uncovered_idx = find(remain_matr(idx,:)==0);
    remain_idx = remain_idx(uncovered_idx);
    remain_matr = remain_matr(uncovered_idx,uncovered_idx);
    N_cover = [N_cover;length(covered_idx)];
    if length(covered_idx)<10
        disp('')
    end
    if isempty(remain_idx)
        break
    end
end
traj_base = traj_pool(picked_idx,1:2*m);
adj_picked = adj(picked_idx,:);