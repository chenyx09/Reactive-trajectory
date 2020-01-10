v=0:0.01:1;
A = zeros(length(v),N_bezier+1);
        for k = 0:N_bezier
            A(:,k+1)=nchoosek(N_bezier,k)*(1-v).^(N_bezier-k).*v.^k;
        end  
        alpha = A\v;