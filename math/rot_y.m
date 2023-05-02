function rot_mat_y = rot_y(rad)

c = cos(rad);
s = sin(rad);

rot_mat_y = [c 0 s;...
             0 1 0;...
             -1*s 0 c];
end