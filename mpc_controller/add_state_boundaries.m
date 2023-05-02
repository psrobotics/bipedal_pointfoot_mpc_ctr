%% lower & upper boundary of eq & ieq constrains

function [boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_p, body_p, ctr_p, path)

addpath(path.casadi);
import casadi.*;

% eq constraint equation Ac = 0 
boundray_v.lbg(1:mpc_c.eq_con_dim) = 0;
boundray_v.ubg(1:mpc_c.eq_con_dim) = 0;

% ineq constraint equation -inf<Ac<0
boundray_v.lbg(mpc_c.eq_con_dim+1: mpc_c.eq_con_dim+mpc_c.ineq_con_dim) = -inf;
boundray_v.ubg(mpc_c.eq_con_dim+1: mpc_c.eq_con_dim+mpc_c.ineq_con_dim) = 0;

% state lower & upper boundary
% state
ub_state = [3*pi*ones(3,1);10*ones(2,1);ctr_p.max_jump_z;...
            5*pi*ones(3,1);50*ones(2,1);ctr_p.max_lift_vel_z];
lb_state = [-3*pi*ones(3,1);-10*ones(2,1);ctr_p.min_dump_z;...
            -5*pi*ones(3,1);-50*ones(3,1)];
ub_state_arr = repmat(ub_state,ctr_p.N+1,1);
lb_state_arr = repmat(lb_state,ctr_p.N+1,1);

% leg force
ub_f_leg = [body_p.m*world_p.g*world_p.fk*50; body_p.m*world_p.g*world_p.fk*50; body_p.max_zforce]; %xyz maximum leg force
lb_f_leg = [-1*body_p.m*world_p.g*world_p.fk*50; -1*body_p.m*world_p.g*world_p.fk*50; 0]; %minimum leg force
ub_f = repmat(ub_f_leg,4,1); % for 4 legs
lb_f = repmat(lb_f_leg,4,1);
ub_f_arr = repmat(ub_f,ctr_p.N,1);
lb_f_arr = repmat(lb_f,ctr_p.N,1);

% foot pos
ub_fp = repmat([0.4;0.4;inf],4,1);
lb_fp = repmat([-0.4;-0.4;-inf],4,1);
ub_fp_arr = repmat(ub_fp,ctr_p.N,1);
lb_fp_arr = repmat(lb_fp,ctr_p.N,1);

% combine lower & upper bounds together, can be changed in each mpc cycle
boundray_v.ubx = [ub_state_arr; ub_f_arr; ub_fp_arr];
boundray_v.lbx = [lb_state_arr; lb_f_arr; lb_fp_arr];

end

