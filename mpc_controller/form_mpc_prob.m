%% Get all constraints in a mpc cycle

% mpc variables, mpc contraints, mpc problem & solver
function [mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_p, body_p, ctr_p, dyn_f, path)

addpath(path.casadi);
import casadi.*;

%% Casadi variables array in one prediction widow
mpc_v.x_arr = SX.sym('x_arr', body_p.state_dim, ctr_p.N+1); % state
mpc_v.f_arr = SX.sym('f_arr', body_p.f_dim, ctr_p.N); % foot force / input
mpc_v.fp_arr = SX.sym('fp_arr', body_p.fp_dim, ctr_p.N); % foot position

%% Reference traj
mpc_v.x_ref_arr = SX.sym('x_ref_arr', body_p.state_dim, ctr_p.N+1); % state
mpc_v.f_ref_arr = SX.sym('f_ref_arr', body_p.f_dim, ctr_p.N); % foot force
mpc_v.fp_ref_arr = SX.sym('fp_ref_arr', body_p.fp_dim, ctr_p.N); % foot position
% contact mat arr, which the leg touches ground, set by fpp_planner
% only foot on ground can output a ground reaction force, for 2 legs
mpc_v.contact_mat_arr = SX.sym('contact_mat_arr', 2, ctr_p.N);

mpc_v.cost_fcn = 0; % the cost function

%% Constraints and cost function
N = ctr_p.N;
state_dim = body_p.state_dim; % number of dim of state, rpy xyz dot_rpy dot_xyz
f_dim = body_p.f_dim; % number of dim of leg force, 3*4
fp_dim = body_p.fp_dim; % number of dim of leg pos, 3*4

% equal constraints
mpc_c.eq_con_dynamic = SX.zeros(state_dim*(N+1),1); % dynamic constraint, 12*(N+1),1
mpc_c.eq_con_foot_contact_ground = SX.zeros(2*N,1); % constraint on leg's motion range, 2*N,1 *
mpc_c.eq_con_foot_non_slip = SX.zeros(fp_dim*N,1); % constraint on foot's non slip stane phase, 6*N,1
mpc_c.eq_con_init_state = mpc_v.x_ref_arr(:,1)-mpc_v.x_arr(:,1); % set init condition constraint, 12,1

mpc_c.eq_con_dim = state_dim*(N+1) + 2*N + fp_dim*N + 12; % dim for eq constraints

% inequal constraints 
mpc_c.ineq_con_foot_range = SX.zeros(2*6*N,1); % ieq constraint on foot's motion range 2 legs*3 axis*2 dir *
mpc_c.ineq_con_foot_friction = SX.zeros(2*4*N,1); % ieq constraint on foot friction 2 legs*2axis*2dir *
mpc_c.ineq_con_zforce_dir = SX.zeros(1*N,1); % z axis force always pointing up
mpc_c.ineq_con_zforce_range = SX.zeros(2*N,1); % z axis force range, swing phase ->0, stance phase -> < max z force, 2 legs *

mpc_c.ineq_con_dim = 2*6*N + 2*4*N + N + 2*N; % dim for ieq constraints

fprintf('%d eq constraints and %d ineq constraints', mpc_c.eq_con_dim, mpc_c.ineq_con_dim);

% define cost function and constraints
for k = 1:N
    x_t = mpc_v.x_arr(:,k); % current state
    x_next = mpc_v.x_arr(:,k+1); %next state
    
    f_t = mpc_v.f_arr(:,k); % current force
    fp_t = mpc_v.fp_arr(:,k); % current foot placement point
    fp_g_t = repmat(x_t(4:6),2,1)+fp_t; %foot pos in global coord * 2 legs
    
    x_ref_t = mpc_v.x_ref_arr(:,k); % current reference state
    f_ref_t = mpc_v.f_ref_arr(:,k); % current reference force
    fp_ref_t = mpc_v.fp_ref_arr(:,k); % current reference foot placement point
    
    contact_mat_t = mpc_v.contact_mat_arr(:,k); % current foot contact point
    dt_t = ctr_p.dt_val(k);
    
    % constraints
    % dynamic equation constraint
    mpc_c.eq_con_dynamic(state_dim*(k-1)+1:state_dim*k) = x_next - (x_t + dyn_f(x_t,f_t,fp_t)*dt_t);
    % zforce direction always point up
    mpc_c.ineq_con_zforce_dir(k) = -1*dot(f_t,repmat([0;0;1],2,1)); % * 2 legs, dot out 1 value
    % zforce range < max z force
    mpc_c.ineq_con_zforce_range(2*(k-1)+1:2*k) = f_t([3,6]) - contact_mat_t.*repmat(body_p.max_zforce,2,1); % * 2 legs
    
    for leg_k = 1:2

        xyz_i = 3*(leg_k-1)+1:3*leg_k; % index for xyz dir
        
        % constrant leg on ground, z=0
        mpc_c.eq_con_foot_contact_ground(2*(k-1)+leg_k) = contact_mat_t(leg_k)*fp_g_t(3*(leg_k-1)+3); % * 2 legs
        
        % keep foot within motion range
        rot_zyx_t = rot_zyx(x_t(1:3));
        hip_pos_global_t = rot_zyx_t*body_p.phip_swing_ref + x_t(4:6);
        leg_vec_t = (fp_g_t(xyz_i) - hip_pos_global_t(:,leg_k)); % leg vector, from hip to foot

        mpc_c.ineq_con_foot_range(2*6*(k-1)+6*(leg_k-1)+1: 2*6*(k-1)+6*leg_k) =...
            body_p.foot_convex_hull*[leg_vec_t;1]; % leg's motion range limit * 2 legs
        
        % ground reaction force within friction cone 
        % * 2 legs *  2 +- dir * 2 axis, xy // 4 = 2 axis * 2 dir
        mpc_c.ineq_con_foot_friction(2*4*(k-1)+2*2*(leg_k-1)+1: 2*4*(k-1)+2*2*leg_k) = world_p.friction_cone*f_t(xyz_i);
        
        % non-slip, if leg touches the ground, ffp_now = ffp_next
        if(k<N)
            fp_g_next = repmat(x_next(4:6),2,1)+mpc_v.fp_arr(:,k+1); % next foot pos in global coord * 2 legs

            mpc_c.eq_con_foot_non_slip(2*3*(k-1)+3*(leg_k-1)+1: 2*3*(k-1)+3*leg_k) =...
                contact_mat_t(leg_k)*(fp_g_next(xyz_i)-fp_g_t(xyz_i)); % * 2 legs * 3 axis, x,y,z
        end
        
    end
    
    % Add up errors to cost fcn
    x_err = x_t - x_ref_t; % state error
    f_err = f_t - f_ref_t; % leg force error
    % foot placement pos error. may change to fp_ref_t with fpp planner *
    fp_err = fp_g_t - fp_ref_t;
    
    % running cost * 2 legs
    mpc_v.cost_fcn = mpc_v.cost_fcn + (x_err'*diag(ctr_p.weight.QX)*x_err...
                           + f_err'*diag(repmat(ctr_p.weight.Qf,2,1))*f_err...
                           + fp_err'*diag(repmat(ctr_p.weight.Qc,2,1))*fp_err) * dt_t;
                       
end

% terminate cost
x_err_f = mpc_v.x_arr(:,N+1)-mpc_v.x_ref_arr(:,N+1);
mpc_v.cost_fcn = mpc_v.cost_fcn + x_err_f'*diag(ctr_p.weight.QN)*x_err_f;

% combine all constraints
mpc_c.constraint_arr = [mpc_c.eq_con_init_state; mpc_c.eq_con_dynamic; mpc_c.eq_con_foot_contact_ground; mpc_c.eq_con_foot_non_slip;...
                        mpc_c.ineq_con_foot_range; mpc_c.ineq_con_foot_friction; mpc_c.ineq_con_zforce_dir; mpc_c.ineq_con_zforce_range];

%% Init the optimal problem, solver
% reform, state, leg force, foot pos into one n*1 vector
mpc_v.opt_variables = [reshape(mpc_v.x_arr, state_dim*(N+1),1);...
                       reshape(mpc_v.f_arr, f_dim*N,1);...
                       reshape(mpc_v.fp_arr, fp_dim*N,1)];
% reform, reference state, leg force, foot pos into one n*1 vector * 2 legs
mpc_v.opt_ref_param =[reshape(mpc_v.x_ref_arr, state_dim*(N+1),1);...
                      reshape(mpc_v.f_ref_arr, f_dim*N,1);...
                      reshape(mpc_v.fp_ref_arr, fp_dim*N,1);...
                      reshape(mpc_v.contact_mat_arr, 2*N,1)];

% construct the mpc problem and solver
mpc_p.nlp_prob = struct('f',mpc_v.cost_fcn, 'x',mpc_v.opt_variables, 'p',mpc_v.opt_ref_param, 'g',mpc_c.constraint_arr);
mpc_p.solver = nlpsol('solver','ipopt',mpc_p.nlp_prob, ctr_p.opt_setting);

%% gen c code


end

