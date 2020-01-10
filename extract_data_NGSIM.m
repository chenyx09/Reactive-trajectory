%%
opts = detectImportOptions('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-1.csv');
data=[readtable('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-1.csv',opts);...
      readtable('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-2.csv',opts);
      readtable('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-3.csv',opts);
      readtable('Next_Generation_Simulation__NGSIM__Vehicle_Trajectories_and_Supporting_Data-4.csv',opts)];

data = data(contains(data.Location,'us-101'),:);
data = sortrows(data,'Global_Time');
%%
% data.Global_Time=data.Global_Time-data.Global_Time(1);
% [a,b]=hist(data.Global_Time,unique(data.Global_Time));
% c=sortrows([a' b],1,'descend');
% f2m = 0.3048;
% lm = linspace(0,21.6,7);
% plot_frame =4;
% tt = c(1,2);
% figure(1)
% 
% clf
% 
% for j =1:plot_frame
% subplot(1,4,j)
% hold on
% seg = data(data.Global_Time==tt+j*200,:);
% for i=1:size(seg,1)
%     draw_rec([seg.Local_X(i),seg.Local_Y(i)]*f2m,[seg.v_Width(i) seg.v_length(i)]*f2m,0,[0,0,1/plot_frame*j]);
% end
% for i=1:length(lm)
%     plot([lm(i),lm(i)],[0,500],'r--')
% end
% axis equal
% end

% obs = calc_obs(seg);