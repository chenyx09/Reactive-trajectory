%% bezier polynomial
% alpha is the coefficient, m * n dim
% s is a vector
function y = bezval(alpha,s)

[m,n] = size(alpha);
if n==6
    y=alpha(1)*(1-s)^5 + alpha(2)*5*s*(1-s)^4 + alpha(3)*10*s^2*(1-s)^3 + alpha(4)*10*s^3*(1-s)^2 + alpha(5)*5*s^4*(1-s) + alpha(6)*s^5;
elseif n==5
    y=alpha(1)*(1-s)^4 + alpha(2)*4*s*(1-s)^3 + alpha(3)*6*s^2*(1-s)^2 + alpha(4)*4*s^3*(1-s) + alpha(5)*s^4;
elseif n==4
    y=alpha(1)*(1-s)^3 + alpha(2)*3*s*(1-s)^2 + alpha(3)*3*s^2*(1-s) + alpha(4)*s^3;
else
    M = n-1;
    sum_term = zeros(m,length(s));
    
    for k = 0:M
        term_k = alpha(:,k+1).*factorial(M)./factorial(k)./factorial(M-k).*s^k*(1-s)^(M-k);
        sum_term = sum_term+term_k;
    end
    
    y = sum_term;
end