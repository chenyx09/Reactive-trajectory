function beta=bezier_square(alpha)
n=length(alpha)-1;
beta=zeros(1,2*n+1);
for i=0:n
    for j=i:n
        if i==j
            beta(i+j+1)=beta(i+j+1)+alpha(i+1)*alpha(j+1)*factorial(n)^2*factorial(i+j)*factorial(2*n-i-j)/factorial(2*n)/factorial(i)/factorial(j)/factorial(n-i)/factorial(n-j);
        else
            beta(i+j+1)=beta(i+j+1)+2*alpha(i+1)*alpha(j+1)*factorial(n)^2*factorial(i+j)*factorial(2*n-i-j)/factorial(2*n)/factorial(i)/factorial(j)/factorial(n-i)/factorial(n-j);
        end
    end
end