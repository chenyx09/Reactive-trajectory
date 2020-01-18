for i=1:size(positive_data_cell{1},1)
a1 = positive_data_cell{1}(i,30+[2,3,4,6,7,9,10,11,12,14,15])./x_norm;
a2 = positive_data_cell{1}(i,[2,3,4,6,7,9,10,11,12,14,15])./x_norm;
pred = traj_predictor(a1',a2');
if pred(1)<0
    disp('')
end
end