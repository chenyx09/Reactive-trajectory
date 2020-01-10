function [x_bar,dx_bar] = bezier_regression_v1(x,t_idx,t0)
%% use bezier polynomial to fit x locally, and generate filtered x and dxdt.

global bez_reg Bez_matr Bez_matr_der Bez_matr_dder N_bezier dB 
N = length(t0);
if size(x,1)<size(x,2)
    x = x';
end
A = Bez_matr{N}(t_idx+1,:);
reg = (A'*A)\(A');
alpha=reg*x;                % get the bezier coefficient with pre-computed regressor
T = t0(end)-t0(1);        
dalpha=dB*alpha;    
x_bar = Bez_matr{N}*alpha;
dx_bar = Bez_matr_der{N}*dalpha/T;




if size(x_bar,1) ==1
    x_bar = x_bar';
end
if size(dx_bar,1) ==1
    dx_bar = dx_bar';
end
end