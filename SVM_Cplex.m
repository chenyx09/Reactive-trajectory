%% Solving SVM
for i=1:M
[status(i),sol(i),obj(i)]=Cplex_SVM(phi_positive{i},phi_negative{i});
obj(i)
end
%%
filename='traj_predictor.m';
f=fopen(filename,'w');
fprintf(f,'function pred = traj_predictor(a1,a2)\n');
fprintf(f,'x=[');
for i=1:size(h,1)
    written = 0;
    for j=1:size(h,2)-1
        if h(i,j)>=1
            if written
                fprintf(f,'*');
            else
                written = 1;
            end
            fprintf(f,'a1(%d)',j);
            if h(i,j)>1
                fprintf(f,'^%d',h(i,j));
            end
        end
    end
    if ~written
        fprintf(f,'1');
    end
    fprintf(f,';');
end
K = i;
for i=1:10
    fprintf(f,'a2(%d);',i);
end
fprintf(f,'a2(11)');
fprintf(f,'];\n');
fprintf(f,'A=[');
for i=1:M
    for j=1:F
        fprintf(f,'%d ',sol(i).w(j));
    end
    if i<=M-1
        fprintf(f,';...\n');
    else
        fprintf(f,'];\n');
    end
end
fprintf(f,'pred=A*x;');
        
    
    

function [status,sol,obj]=Cplex_SVM(phi_p,phi_n)
global cost_type slack_max x_norm gamma

if isempty(cost_type)
    cost_type = 'L1';
end


d = size(phi_p,2);
[n_p,F] = size(phi_p);
n_n = size(phi_n,1);
numberOfCores = 4;

num.w = F;

% num.slack_p = n_p;

num.slack_np = n_n;
num.slack_nn = n_n;

num.slack_max = 1;

var_name = {'w', 'slack_np','slack_nn' ,'slack_max'};
nvar = 0;

for v = var_name
    var = v{1};
    ndx.(var) = nvar + (1:num.(var));
    nvar = nvar + num.(var);
end
% ndx.b_s_p = reshape(ndx.b_s_p,n_class,n_p);
% ndx.b_s_n = reshape(ndx.b_s_n,n_class,n_n);



% ndx.slack = reshape(ndx.slack,n_class,n_n);

% options = cplexoptimset('Display', 'off');
cplex = Cplex('SVM');
cplex.DisplayFunc=[];
obj = zeros(nvar,1);
if strcmp(cost_type,'L1')
    obj(reshape(ndx.slack_nn,[],1))=ones(num.slack_nn,1);
    obj(reshape(ndx.slack_np,[],1))=1*ones(num.slack_np,1);
elseif strcmp(cost_type,'Linf')
    obj(ndx.slack_max)=5000;
elseif strcmp(cost_type,'mixed')
    obj(reshape(ndx.slack_nn,[],1))=ones(num.slack_nn,1);
    obj(reshape(ndx.slack_np,[],1))=50*ones(num.slack_np,1);
    obj(ndx.slack_max)=300*n_n;
else
    error('cost type not supported')
end
lb = -inf*ones(nvar,1);
ub = inf*ones(nvar,1);
lb(ndx.slack_np)=zeros(num.slack_np,1);
ub(ndx.slack_nn)=100*zeros(num.slack_nn,1);

% if strcmp(cost_type,'L1')
%     ub(ndx.slack_np)=slack_max*ones(num.slack_np,1);
% end
lb(ndx.w(1))=-10;
ub(ndx.w(1))=10;

% lb(reshape(ndx.lambda,[],1))=zeros(num.lambda,1);
cplex.addCols(obj,[],lb,ub);
cplex.Param.threads.Cur = numberOfCores;
cplex.Param.parallel.Cur =1;
% cplex.Param.mip.display.Cur=0;
% cplex.Param.timelimit.Cur = TIMEOUTE_LIMIT;
% cplex.Param.timelimit.Cur = max(TIMEOUTE_LIMIT,n_p/5);
cplex.Param.output.clonelog.Cur=0;

Q = sparse(nvar,nvar);
Q(ndx.w(2:end),ndx.w(2:end))=eye(F-1);
cplex.addQCs(sparse(nvar,1), Q, 'L', 1.0);


A = sparse(n_p,nvar);
A(:,ndx.w)=phi_p;
cplex.addRows(zeros(n_p,1),A,inf*ones(n_p,1));

A = sparse(n_n,nvar);
A(:,ndx.w)=phi_n;
A(:,ndx.slack_np) = -speye(n_n);
A(:,ndx.slack_nn) = -speye(n_n);
cplex.addRows(zeros(n_n,1),A,zeros(n_n,1));

if strcmp(cost_type,'Linf')
A = sparse(num.slack_np,nvar);
A(:,ndx.slack_np)=speye(num.slack_np);
A(:,ndx.slack_nn)=speye(num.slack_np);
A(:,ndx.slack_max) = -ones(num.slack_nn,1);
cplex.addRows(-inf*ones(num.slack_nn,1),A,zeros(num.slack_nn,1));
end




cplex.solve();
status = cplex.Solution.status;
if ~isempty(cplex.Solution.x)
    sol0 = cplex.Solution.x;
    
    for v = var_name
        var = v{1};
        sol.(var) = sol0(ndx.(var));
        obj=cplex.Solution.objval;
    end
    %     sol.slack_p = (phi_p*sol.w(:,1).*sigma_p+phi_p*sol.w(:,2).*(1-sigma_p))';
    %     sol.slack_nnnnnnn = (phi_n*sol.w(:,1).*sigma_n+phi_n*sol.w(:,2).*(1-sigma_n))';
    
    %     sol.class_sense=class_sense;
else
    sol=[];
    obj=inf;
end
end














