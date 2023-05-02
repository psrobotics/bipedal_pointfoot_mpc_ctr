function s_mat = skew_mat(vec_)

% generate a skew-symmetric matrix for 3d vec's corss product
% [a]x b = a x b

a1 = vec_(1);
a2 = vec_(2);
a3 = vec_(3);

s_mat = [0 -1*a3 a2;...
         a3 0 -1*a1;...
         -1*a2 a1 0];

end