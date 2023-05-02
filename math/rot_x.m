function rot_mat_x = rot_x(rad)

c = cos(rad);
s = sin(rad);

rot_mat_x = [1 0 0;...
             0 c -1*s;...
             0 s  c];
end