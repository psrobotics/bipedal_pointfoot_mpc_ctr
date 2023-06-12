function [world, body, ctr, path] = hardware_params()
%% Casadi path
% Change to your casadi path
path.casadi = 'D:\matlab_lib\casadi-3.6.0-windows64-matlab2018b';

%% Simulation params
world.fk = 0.5; % friction
world.g = 9.81; % gravity constant

world.friction_cone = [1/world.fk, 0 -1;...
                      -1/world.fk, 0 -1;...
                      0, 1/world.fk, -1;...
                      0, -1/world.fk, -1];

%% Controller params
ctr.phase_num = 4;
ctr.N = 80*3; % mpc period window
ctr.T = 3*3; % mpc period time
ctr.dt_val = (ctr.T/ctr.N) .* ones(ctr.N,1); % dt vector

ctr.max_jump_z = 0.55; % max jumping height, constraints
ctr.min_dump_z = 0.15; % min standing height
ctr.max_lift_vel_z = 6.5; % max jumping velocity
ctr.init_z = 0.3;

ctr.x_init_tar_val = [0; 0; 0; 0; 0; ctr.init_z]; % init state
ctr.dx_init_tar_val = [0; 0; 0; 0; 0; 0]; % init d state
ctr.x_final_tar_val = [0; 0; 0.2*9*0; 0.4*9*1.5; 0.4*9*0; 0.3]; % final target state r p y; x y z
ctr.dx_final_tar_val = [0; 0; 0.2*0; 0.4*1.5; 0.4*0; 0];

%ctr.contact_state_ratio = ctr.N.*[0.35 0.15 0.475 0.025]; % pull, jump, flight, impact
ctr.contact_state_ratio = ctr.N.*[1/48 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12]; % pull, jump, flight, impact

% ctr.contact_state_val = [ones(2, ctr.contact_state_ratio(1)),...
%                              0 * ones(2, ctr.contact_state_ratio(2)),...
%                              ones(2, ctr.contact_state_ratio(3)),...
%                              0 * ones(2, ctr.contact_state_ratio(4)),...
%                              ones(2, ctr.contact_state_ratio(1)),...
%                              0 * ones(2, ctr.contact_state_ratio(2)),...
%                              ones(2, ctr.contact_state_ratio(3)),...
%                              0 * ones(2, ctr.contact_state_ratio(4)),...
%                              ones(2, ctr.contact_state_ratio(1)),...
%                              0 * ones(2, ctr.contact_state_ratio(2)),...
%                              ones(2, ctr.contact_state_ratio(3)),...
%                              0 * ones(2, ctr.contact_state_ratio(4))]; % no foot contact during last 2 phases

ctr.gait_num = 12;
ctr.contact_state_val = [repmat([1;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;0], 1, ctr.contact_state_ratio(1))];
ctr.contact_state_val = repmat(ctr.contact_state_val, 1, ctr.gait_num);
% no foot contact during last 2 phases
                 
% cost gains
ctr.weight.QX = [10 10 0, 10 10 10, 10 10 10, 10 10 10 ]'; % state error
ctr.weight.QN = [10 10 0, 50 50 50, 10 10 10, 10 10 10 ]'; % state error, final
ctr.weight.Qc = 100*[50 50 50]'; % foot placement error on 3 axis
ctr.weight.Qf = [0.1 0.1 0.1]'; % input error on 3 axis

% casadi optimal settings
ctr.opt_setting.expand =true;
ctr.opt_setting.ipopt.max_iter=1500;
ctr.opt_setting.ipopt.print_level=0;
ctr.opt_setting.ipopt.acceptable_tol=1e-4;
ctr.opt_setting.ipopt.acceptable_obj_change_tol=1e-6;
ctr.opt_setting.ipopt.tol=1e-4;
ctr.opt_setting.ipopt.nlp_scaling_method='gradient-based';
ctr.opt_setting.ipopt.constr_viol_tol=1e-3;
ctr.opt_setting.ipopt.fixed_variable_treatment='relax_bounds';

%% Robot params

body.state_dim = 12; % number of dim of state, rpy xyz dot_rpy dot_xyz
body.f_dim = 6; % number of dim of leg force, 3*2, 2 leg
body.fp_dim = 6; % number of dim of leg pos, 3*2, 2 leg


body.m = 5;
body.i_vec = [0.06 0.1 0.05]*2;
body.i_mat = [body.i_vec(1) 0 0;... % roll
              0 body.i_vec(2) 0;... % pitch
              0 0 body.i_vec(3)]; % yaw
       
%body.length = 0.34;
body.width = 0.3; % distance between 2 legs

% leg length
body.l1 = 0.2;
body.l2 = 0.17;

% foot motion range, in m
body.foot_x_range = 0.15;
body.foot_y_range = 0.15;
body.foot_z_range = 0.3;

% output force range
body.max_zforce = 100;

% calaute 2 hip positions
body.hip_vec = [0; body.width/2; 0];
body.hip_dir_mat = [0 0; 1 -1; 0 0]; % 3,2 hip vec
body.hip_pos = body.hip_dir_mat .* repmat(body.hip_vec,1,2); % 3,2 hip pos vec
body.foot_pos = repmat([0; 0; -0.6*ctr.init_z],1,2); % init foot pos

body.phip_swing_ref = body.hip_pos + body.foot_pos;
% ref foot pos at swing phase

body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1);

% the range foot can move within
body.foot_convex_hull = [1 0 0 -body.foot_x_range;
                        -1 0 0 -body.foot_x_range;
                        0 1 0 -body.foot_y_range;
                        0 -1 0 -body.foot_y_range;
                        0 0 1 -ctr.min_dump_z;
                        0 0 -1 -body.foot_z_range];
                

end