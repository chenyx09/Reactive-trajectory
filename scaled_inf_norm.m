function d = scaled_inf_norm(traj1,traj2)
m = length(traj1)/2;
x_scaling = (m-1)./(m-1:2*m-2);
y_scaling = (m-1)./(m-1:2*m-2)/4;
traj1 = [traj1(1:m).*y_scaling;traj1(m+1:2*m).*x_scaling];
traj2 = [traj2(1:m).*y_scaling;traj2(m+1:2*m).*x_scaling];
d = max(vecnorm(traj1-traj2));