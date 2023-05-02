function rot_mat_z = rot_z(rad)

c = cos(rad);
s = sin(rad);

rot_mat_z = [c -1*s 0;...
             s c 0;...
             0 0 1];
end