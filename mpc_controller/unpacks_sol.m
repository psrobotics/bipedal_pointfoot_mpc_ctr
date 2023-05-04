function [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_p, ctr_p, path)

addpath(path.casadi);
import casadi.*;

% unpack data from casadi result 
state_dim = body_p.state_dim;
f_dim = body_p.f_dim;
fp_dim = body_p.fp_dim;
N = ctr_p.N;

x_sol=sol.x(1:state_dim*(N+1));
x_sol=reshape(full(x_sol),state_dim,(N+1));% COM under world coord

f_sol=sol.x(state_dim*(N+1)+1:state_dim*(N+1)+f_dim*N);
f_sol=reshape(full(f_sol),f_dim,N);% foot force

fp_l_sol=sol.x(state_dim*(N+1)+f_dim*N+1:state_dim*(N+1)+f_dim*N+fp_dim*N);
% Foot pos in rbt coord
fp_l_sol=reshape(full(fp_l_sol),f_dim,N);

% Foot pos in world coord. * 2 legs
fp_g_sol=fp_l_sol+repmat(x_sol(4:6,1:end-1),2,1);

end

