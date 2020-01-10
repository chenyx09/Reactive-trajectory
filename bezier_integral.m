function m = bezier_integral(alpha)
n=length(alpha)-1;
m=0;
for i=0:n
    m = m + alpha(1+i) * nchoosek(n,i)*gamma(1+i)*gamma(n+1-i)/gamma(n+2);
end