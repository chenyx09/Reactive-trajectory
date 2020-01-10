function draw_rec(c,s,psi,color)
if nargin<=2
    color='r';
end
if size(c,1)==1
    c=c';
end
w=s(2)/2;
a=s(1)/2;
b=s(1)/2;
t=[a*cos(psi)+w*sin(psi) a*cos(psi)-w*sin(psi) -b*cos(psi)-w*sin(psi) -b*cos(psi)+w*sin(psi) a*cos(psi)+w*sin(psi);...
   w*cos(psi)-a*sin(psi) -w*cos(psi)-a*sin(psi) -w*cos(psi)+b*sin(psi) w*cos(psi)+b*sin(psi) w*cos(psi)-a*sin(psi)]+repmat(c,1,5);
plot(t(1,:),t(2,:),'linewidth',2,'color',color);