% N_p = size(positive_data,1);
% h = mss_asd(14,2);
% 
% Fh = size(h,1);
% F = Fh+39;
% x_norm = max(abs(positive_data(:,[2,3,4,6,7,8,9,10,11,12,13,14,15])));
% parfor i=1:1
%     i
%     phi_positive{i} = [];
%     phi_negative{i} = [];
%     for j=1:size(positive_data_cell{i},1)
%         xx=[positive_data_cell{i}(j,30+[2,3,4,6,7,8,9,10,11,12,13,14,15]) 1]./[x_norm 1];
%         entry=zeros(1,Fh);
%         for k=1:size(h,1)
%             entry(k)=prod(xx.^h(k,:));
%         end
% %         entry(k+1:k+11)=positive_data_cell{i}(j,[2,3,4,6,7,9,10,11,12,14,15])./x_norm;
%         phi_positive{i}=[phi_positive{i};[entry tanh(3*positive_data_cell{i}(j,30+[2,3,4,6,7,8,9,10,11,12,13,14,15])./x_norm)...
%           exp(-(positive_data_cell{i}(j,30+[2,3,4,6,7,8,9,10,11,12,13,14,15])./x_norm).^2)  positive_data_cell{i}(j,[2,3,4,6,7,8,9,10,11,12,13,14,15])./x_norm]];
%     end
%     for j=1:size(negative_data_cell{i},1)
%         xx=[negative_data_cell{i}(j,30+[2,3,4,6,7,8,9,10,11,12,13,14,15]) 1]./[x_norm 1];
%         entry=zeros(1,Fh);
%         for k=1:size(h,1)
%             entry(k)=prod(xx.^h(k,:));
%         end
% %         entry(k+1:k+11)=negative_data_cell{i}(j,[2,3,4,6,7,9,10,11,12,14,15])./x_norm;
%         phi_negative{i}=[phi_negative{i};[entry tanh(3*negative_data_cell{i}(j,30+[2,3,4,6,7,8,9,10,11,12,13,14,15])./x_norm)...
%             exp(-(negative_data_cell{i}(j,30+[2,3,4,6,7,8,9,10,11,12,13,14,15])./x_norm).^2) negative_data_cell{i}(j,[2,3,4,6,7,8,9,10,11,12,13,14,15])./x_norm]];
%     end
% end

for i=1:M
    negative_data_cell{i} = sortrows(negative_data_cell{i},46,'ascend');
end