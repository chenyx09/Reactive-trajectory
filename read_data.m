opts = detectImportOptions('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-11.csv');
data=readtable('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-11.csv',opts);
data = data(contains(data.Location,'i-80'),:);
data = sortrows(data,'Global_Time');
data.Global_Time=data.Global_Time-data.Global_Time(1);
[a,b]=hist(data.Global_Time,unique(data.Global_Time));
c=sortrows([a' b],1,'descend');
f2m = 0.3048;
lm = linspace(0,21.6,7);
figure(1)
clf
hold on
for j =1:3
tt = c(j,2);
seg = data(data.Global_Time==tt,:);

for i=1:size(seg,1)
    draw_rec([seg.Local_X(i),seg.Local_Y(i)]*f2m,[seg.v_Width(i) seg.v_length(i)]*f2m,0,'b');
end
end
for i=1:length(lm)
    plot([lm(i),lm(i)],[0,500],'r--')
end
axis equal
