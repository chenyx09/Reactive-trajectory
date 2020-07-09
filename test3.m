v_profile=zeros(17,7);

for i=1:size(traj_base,1)
    
    for j=2:7
        v_profile(i,j)=(traj_base(i,j)-traj_base(i,j-1))/0.5;
    end
end
traj_base1 = [traj_base v_profile];