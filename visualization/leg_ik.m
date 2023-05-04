function [j_rad_arr, j_pos_arr] = leg_ik(leg_vec,l1,l2)
% get joint rad and pos for 3dof leg with a input leg vec

x = leg_vec(1);
y = leg_vec(2);
z = leg_vec(3);

d_yz = sqrt(y^2+z^2);
d_xyz = sqrt(x^2+y^2+z^2);
j_rad_arr.a1 = atan2(z,y);

b1 = acos((l1^2+d_xyz^2-l2^2)/(2*l1*d_xyz));
b2 = atan2( -1*d_yz,x );
j_rad_arr.a2 = b1+b2;

b3 = ( (l1^2+l2^2-d_xyz^2)/(2*l1*l2) );
j_rad_arr.a3 = pi+j_rad_arr.a2-b3;

j_pos_arr.hip = [0;0;0];
d1 = l1*sin(j_rad_arr.a2);
j_pos_arr.knee = [l1*cos(j_rad_arr.a2); -1*d1*cos(j_rad_arr.a1); -1*d1*sin(j_rad_arr.a1)];
j_pos_arr.foot = leg_vec;

end