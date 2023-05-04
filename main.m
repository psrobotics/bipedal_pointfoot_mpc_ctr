%% Add casadi path

clc; clear;
close all; warning off;

%% Get hardware params & set package path
[world_params, body_params, ctr_params, path] = hardware_params();

addpath(path.casadi);
import casadi.*;

addpath('math\');
addpath('srb_dynamics\');
addpath('mpc_controller\');
addpath('fpp_planner\');
addpath('visualization\');

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_params, body_params, path);

%% Form the mpc problem
[mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_params, body_params, ctr_params, dyn_f, path);
%%
[boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_params, body_params, ctr_params, path);
%%
[ref_traj_v] = fpp_planner(world_params, body_params, ctr_params, path);

%% Slove the NLP prob
sol = mpc_p.solver('x0',ref_traj_v.x0,...
                   'lbx',boundray_v.lbx,...
                   'ubx',boundray_v.ubx,...
                   'lbg',boundray_v.lbg,...
                   'ubg',boundray_v.ubg,...
                   'p',ref_traj_v.p);
               
  %% Unpack data
  [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_params, ctr_params, path);
  
  %%
  rbt_anime(x_sol,f_sol,fp_g_sol,ref_traj_v,ctr_params.T,ctr_params.N,body_params);
  
%   %% Plots
%  clf;
%  subplot(1,2,1);
% plot(x_sol(4,:),"LineWidth",1.5);
% hold on;
% plot(x_sol(5,:),"LineWidth",1.5);
% hold on;
% plot(x_sol(6,:),"LineWidth",1.5);
% hold on;
% title("Body's Position")
% xlabel("Step")
% ylabel("m")
% legend("Position X","Position Y","Position Z");
% grid on  
% 
%  subplot(1,2,2);
%  plot(x_sol(1,:),"LineWidth",1.5);
% hold on;
% plot(x_sol(2,:),"LineWidth",1.5);
% hold on;
% plot(x_sol(3,:),"LineWidth",1.5);
% hold on;
% title("Body's Euler Angle")
% xlabel("Step")
% ylabel("rad")
% legend("Roll","Pitch","Yaw");
% grid on  
% 
% %% velocity plot
% clf;
% plot(x_sol(9,:),"LineWidth",1.5);
% hold on;
% plot(x_sol(10,:),"LineWidth",1.5);
% hold on;
% title("Body's Velocity")
% xlabel("Step")
% ylabel("m/s or rad/s")
% legend("Yaw Velocity rad/s","X Velocity m/s");
% grid on  
% 
% %% fpp
% clf;
%  subplot(1,2,1);
% plot(fp_g_sol(1,:),"LineWidth",1.5);
% hold on;
% plot(fp_g_sol(2,:),"LineWidth",1.5);
% hold on;
% plot(fp_g_sol(3,:),"LineWidth",1.5);
% hold on;
% title("Foot Placement Point global of FL leg")
% xlabel("Step")
% ylabel("m")
% legend("Position X","Position Y","Position Z");
% grid on
% 
%  subplot(1,2,2);
%  plot(f_sol(1,:),"LineWidth",1.5);
% hold on;
% plot(f_sol(2,:),"LineWidth",1.5);
% hold on;
% plot(f_sol(3,:),"LineWidth",1.5);
% hold on;
% title("Ground Reaction Force of FL leg")
% xlabel("Step")
% ylabel("N")
% legend("Force X","Force Y","Force Z");
% grid on  
