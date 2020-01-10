


% x = load('robot_data');
% phi_p = x.phi_positive;
% phi_n = x.phi_negative;


% threshold_x = 0;
n_p = size(phi_positive,1);
n_n = size(phi_negative,1);


% disp(['number of positive mismatch = ',num2str(length(pos_mis))])
% disp(['number of negative mismatch = ',num2str(length(neg_mis))])
% if ~isempty(sol)
% extract_reactive_bound_2region(sol);
% else
%     disp('failure')
% end
% pos_mis = [];
% for i=1:n_p
%     x = phi_p(i,x_idx)'.*x_norm(1:4);
%     u = phi_p(i,u1_idx)'.*x_norm(5:6);
%     [A,b]=const_para(x);
%     if ~all(A*u<=b)
%         pos_mis=[pos_mis;i];
%     end
% end
% neg_mis = [];
% for i=1:n_n
%     x = phi_n(i,x_idx)'.*x_norm(1:4);
%     u = phi_n(i,u1_idx)'.*x_norm(5:6);
%     [A,b]=const_para(x);
%     if all(A*u<=b)
%         neg_mis=[neg_mis;i];
%     end
% end


[status,sol,obj]=Cplex_SVM(phi_positive,phi_negative)

function [status,sol,obj]=Cplex_SVM(phi_p,phi_n)
global cost_type slack_max x_norm gamma M F

if isempty(cost_type)
    cost_type = 'L1';
end


d = size(phi_p,2);
n_p = size(phi_p,1);
n_n = size(phi_n,1);
numberOfCores = 4;

num.w = F*M;

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
w_vec_idx = ndx.w;
ndx.w = reshape(ndx.w,F,M);

% ndx.slack = reshape(ndx.slack,n_class,n_n);

% options = cplexoptimset('Display', 'off');
cplex = Cplex('SVM');
cplex.DisplayFunc=[];
obj = zeros(nvar,1);
if strcmp(cost_type,'L1')
    obj(reshape(ndx.slack_nn,[],1))=ones(num.slack_nn,1);
    obj(reshape(ndx.slack_np,[],1))=100*ones(num.slack_np,1);
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
ub(ndx.slack_nn)=zeros(num.slack_nn,1);

% if strcmp(cost_type,'L1')
%     ub(ndx.slack_np)=slack_max*ones(num.slack_np,1);
% end
for i=1:M
    lb(ndx.w(1,i))=-10;
    ub(ndx.w(1,i))=10;
end

% lb(reshape(ndx.lambda,[],1))=zeros(num.lambda,1);
cplex.addCols(obj,[],lb,ub);
cplex.Param.threads.Cur = numberOfCores;
cplex.Param.parallel.Cur =1;
% cplex.Param.mip.display.Cur=0;
% cplex.Param.timelimit.Cur = TIMEOUTE_LIMIT;
% cplex.Param.timelimit.Cur = max(TIMEOUTE_LIMIT,n_p/5);
cplex.Param.output.clonelog.Cur=0;

for i=1:M
    Q = sparse(nvar,nvar);
    Q(ndx.w(2:end,i),ndx.w(2:end,i))=eye(F-1);
    cplex.addQCs(sparse(nvar,1), Q, 'L', 1.0);
end


A = sparse(n_p,nvar);
A(:,w_vec_idx)=phi_p;
cplex.addRows(zeros(n_p,1),A,inf*ones(n_p,1));

A = sparse(n_n,nvar);
A(:,w_vec_idx)=phi_n;
A(:,ndx.slack_np) = -speye(n_n);
A(:,ndx.slack_nn) = -speye(n_n);
cplex.addRows(zeros(n_n,1),A,zeros(n_n,1));

% if strcmp(cost_type,'Linf')
A = sparse(num.slack_np,nvar);
A(:,ndx.slack_np)=speye(num.slack_np);
A(:,ndx.slack_nn)=speye(num.slack_np);
A(:,ndx.slack_max) = -ones(num.slack_nn,1);
cplex.addRows(-inf*ones(num.slack_nn,1),A,zeros(num.slack_nn,1));
% end




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














