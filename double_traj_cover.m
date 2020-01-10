function [cover_set,cover_score] = double_traj_cover(traj_base,traj,delta)
N = size(traj_base,1);
m = length(traj)/2;
x_scaling = (m-1)./(m-1:2*m-2);
y_scaling = (m-1)./(m-1:2*m-2)/4;
traj0 = [traj(1:m).*y_scaling;traj(m+1:2*m).*x_scaling];
cover_set=[];
cover_score = [];
for i=1:N-1
    traj1 = [traj_base(i,1:m).*y_scaling;traj_base(i,m+1:2*m).*x_scaling];
    for j=i+1:N
        traj2 = [traj_base(j,1:m).*y_scaling;traj_base(i,m+1:2*m).*x_scaling];
        fail=0;
        for t=2:m
            x0 = traj0(:,t)-traj2(:,t);
            x1 = traj2(:,t)-traj1(:,t);
            k = min(max((x0'*x1)/(x1'*x1),0),1);
            x_star = k*x1+x0;
            if norm(x_star)>delta
                fail=1;
                break
            end
        end
        if ~fail
            cover_set = [cover_set;i,j];
            cover_score = [cover_score;scaled_inf_norm(traj_base(i,:),traj_base(j,:))];
        end
    end
end
            