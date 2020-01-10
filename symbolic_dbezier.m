function A=symbolic_dbezier(n)
% alpha_df=A*alpha_f
A=zeros(n,n+1);
for i=1:n
    A(i,i)=-n;
    A(i,i+1)=n;
end

