function q = proj_IK_with_slider(p, orient, c)
    % 
    % theta_ref = clip(theta_ref, -pi, pi);
    % r_proj_ref = clip(r_proj_ref, 0.05, 0.2);
    % z_ref = clip(z_ref, -0.05, 0.13);

    % prismatic far extreme position: 2389 (+30deg = offset) (0deg w/o offset)
    % prismatic close extreme position: 1536 (-45deg) (-75deg w/o offset)

    % Split reachable space in half
    x = p(1);
    y = p(2);
    z = p(3);
    max_delta_d = c(5);
    if y > (max_delta_d/2)
        % set to close extreme prismatic position
        q_pris = -pi/4 - pi/6;
        y_new = y - max_delta_d;
    else
        % set to far extreme prismatic position
        q_pris = 0;
        y_new = y;
    end
    q = proj_IK([x, y_new, z], orient, c);
    q = [q, q_pris];
end