function b=bezier_base(n,i,s)
b=factorial(n)/factorial(i)/factorial(n-i)*s^i*(1-s)^(n-i);
end