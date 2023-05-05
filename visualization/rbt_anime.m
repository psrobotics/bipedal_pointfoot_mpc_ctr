function [] = rbt_anime(x_arr,f_arr,fp_arr,ref_traj_v,T,N,body_p)

size_arr = size(x_arr);
len = size_arr(2);

figure(1);

 v = VideoWriter('3d_walk_4','MPEG-4');
 v.FrameRate = N/T;
 open(v);

for k = 1:1:len-1

    x_t = x_arr(:,k);

    fp_w_1 = fp_arr(1:3,k);
    fp_w_2 = fp_arr(4:6,k);

    ref_fp_w_1 = ref_traj_v.fp_ref_val(1:3,k);
    ref_fp_w_2 = ref_traj_v.fp_ref_val(4:6,k);
    
    r_mat = rot_zyx(x_t(1:3));
    
    foot_pos = fp_arr(:,k);
    feetforce_used = f_arr(:,k)*0.5;

    % get leg ik info
    hip_g_r = x_t(4:6) + r_mat * [0; 0.3/2; 0];
    hip_g_l = x_t(4:6) + r_mat * [0; -0.3/2; 0];
    leg_vec_r = fp_w_1 - hip_g_r;
    leg_vec_l = fp_w_2 - hip_g_l;

    [j_r_r, j_p_r] = leg_ik(leg_vec_r,0.29,0.27);
    [j_r_l, j_p_l] = leg_ik(leg_vec_l,0.29,0.27);

    p_knee_g_r = hip_g_r + r_mat*j_p_r.knee;
    p_knee_g_l = hip_g_l + r_mat*j_p_l.knee;

    p_leg_arr_r = [x_t(4:6), hip_g_r, p_knee_g_r, fp_w_1];
    p_leg_arr_l = [x_t(4:6), hip_g_l, p_knee_g_l, fp_w_2];
    
    clf;
    hold on;
    axis equal;
    grid on;
    
    view(-34,33);
    %view(90,0)
    axis([-0.5,4,-1,1,0,1.2]);
    
    plot_cube(r_mat,0.15,0.25,0.4,x_t(4:6)+r_mat*[0;0;0.2]);

    % leg 
    plot3(p_leg_arr_r(1,:), p_leg_arr_r(2,:), p_leg_arr_r(3,:), 'black','linewidth',2);
    plot3(p_leg_arr_l(1,:), p_leg_arr_l(2,:), p_leg_arr_l(3,:), 'black','linewidth',2);

    % foot
    plot3(fp_w_1(1),fp_w_1(2),fp_w_1(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(fp_w_2(1),fp_w_2(2),fp_w_2(3),'o','linewidth',2,'color','b','markersize',6);

    % hip
    plot3(hip_g_r(1),hip_g_r(2),hip_g_r(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(hip_g_l(1),hip_g_l(2),hip_g_l(3),'o','linewidth',2,'color','b','markersize',6);

    % knee
    plot3(p_knee_g_r(1),p_knee_g_r(2),p_knee_g_r(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(p_knee_g_l(1),p_knee_g_l(2),p_knee_g_l(3),'o','linewidth',2,'color','b','markersize',6);

    % ref foot
    plot3(ref_fp_w_1(1),ref_fp_w_1(2),ref_fp_w_1(3),'o','linewidth',2,'color','g','markersize',4);
    plot3(ref_fp_w_2(1),ref_fp_w_2(2),ref_fp_w_2(3),'o','linewidth',2,'color','g','markersize',4);
    
    % * 2 legs
    for i=1:2
        x_indx=3*(i-1)+1;
        y_indx=3*(i-1)+2;
        z_indx=3*(i-1)+3;
        
         plot3([foot_pos(x_indx),foot_pos(x_indx)+0.01*feetforce_used(x_indx)],...
            [foot_pos(y_indx),foot_pos(y_indx)+0.01*feetforce_used(y_indx)],...
            [foot_pos(z_indx),foot_pos(z_indx)+0.01*feetforce_used(z_indx)],'linewidth',1.1,'color','r');
        
    end
    
    pause(T/N);
    hold on;
    
      frame = getframe(gcf);
      writeVideo(v,frame);
    
end

 close(v);

end

