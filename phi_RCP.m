function res = phi_RCP(epsilon,q,N)
res=0;
if q<50
for i=0:q
    res = res + epsilon^i * nchoosek(N,i)* (1-epsilon)^(N-i);
    if i>80
        disp('')
    end
    if isnan(res)
        disp('')
    end
end
else
    res = normcdf(q,epsilon*N,sqrt(N*epsilon*(1-epsilon)));
end