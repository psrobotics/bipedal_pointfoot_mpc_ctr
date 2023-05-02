function [ref_traj_v] = fpp_planner(world_p, body_p, ctr_p, path)

addpath(path.casadi);
import casadi.*;

%% Generate reference trajectory
fp_ref_i = diag([1 1 1, 1 -1 1, -1 1 1, -1 -1 1])*repmat([0.4 0.2 -1*ctr_p.init_z],1,4)'; % foot pos ref

ref_traj_v.x_ref_val = zeros(12,ctr_p.N+1);
ref_traj_v.f_ref_val = zeros(12,ctr_p.N);
ref_traj_v.fp_ref_val = zeros(12,ctr_p.N);

for i = 1:6
    ref_traj_v.x_ref_val(i,:) = linspace(ctr_p.x_init_tar_val(i),ctr_p.x_final_tar_val(i),ctr_p.N+1); %rpy xyz
    ref_traj_v.x_ref_val(i+6,:) = linspace(ctr_p.dx_init_tar_val(i),ctr_p.dx_final_tar_val(i),ctr_p.N+1); %velocity
end

% spine on the z axis
s_a = [ref_traj_v.x_ref_val(4,1),ref_traj_v.x_ref_val(4,ctr_p.N/2),ref_traj_v.x_ref_val(4,ctr_p.N)]; % x axis
s_b = [ctr_p.x_init_tar_val(6),ctr_p.x_final_tar_val(6),ctr_p.x_init_tar_val(6)+0]; % z axis
ref_traj_v.x_ref_val(6,:) = interp1(s_a,s_b,ref_traj_v.x_ref_val(4,:),'spline');


for leg_i = 1:4
    for xyz_j = 1:3
        %u_ref_val(3*(leg_i-1)+xyz_j,:) = x_ref_val(xyz_j:end-1) + fp_ref_i(3*(leg_i-1)+xyz_j);
        ref_traj_v.fp_ref_val(3*(leg_i-1)+xyz_j,:) = fp_ref_i(3*(leg_i-1)+xyz_j);
    end
end

% combine all ref traj
ref_traj_v.p = [reshape(ref_traj_v.x_ref_val,body_p.state_dim*(ctr_p.N+1),1);...
                reshape(ref_traj_v.f_ref_val,body_p.f_dim*ctr_p.N,1);...
                reshape(ref_traj_v.fp_ref_val,body_p.fp_dim*ctr_p.N,1);...
                reshape(ctr_p.contact_state_val,4*ctr_p.N,1)];
      
ref_traj_v.x0 = [reshape(ref_traj_v.x_ref_val,body_p.state_dim*(ctr_p.N+1),1);...
                 reshape(ref_traj_v.f_ref_val,body_p.f_dim*ctr_p.N,1);...
                 reshape(ref_traj_v.fp_ref_val,body_p.fp_dim*ctr_p.N,1)]; % initial states
      
end

