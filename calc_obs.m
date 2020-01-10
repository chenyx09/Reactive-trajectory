function obs = calc_obs(frame)

n_lanes = 6;
for i=1:n_lanes
    obs{i}=[];
end
for i=1:size(frame,1)

    lane_ID = frame.Lane_ID(i);
    ub = frame.Local_Y(i);
    lb = ub - frame.v_length(i);
    left_X = frame.Local_X(i)-frame.v_Width(i)/2;
    right_X = frame.Local_X(i)+frame.v_Width(i)/2;
    obs{lane_ID}=[obs{lane_ID};lb ub left_X right_X];
end
for i=1:n_lanes
    if ~isempty(obs{i})
        obs{i}=sortrows(obs{i},1);
    end
end
for i=1:n_lanes
    if isempty(obs{i})
        obs{i}=[-100 -99 i*3.6-1.8-0.9 i*3.6-1.8+0.9];
    end
end