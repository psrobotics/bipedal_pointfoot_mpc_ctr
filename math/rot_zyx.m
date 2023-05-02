function rot_mat = rot_zyx(vec_)

psi_ = vec_(3);
theta_ = vec_(2);
phi_ = vec_(1);

%z-psi-偏航-yaw
%y-theta-俯仰-pitch
%x-phi-横滚-roll

rot_mat_z = rot_z(psi_);
rot_mat_y = rot_y(theta_);
rot_mat_x = rot_x(phi_);

rot_mat = rot_mat_z * rot_mat_y * rot_mat_x;

end
