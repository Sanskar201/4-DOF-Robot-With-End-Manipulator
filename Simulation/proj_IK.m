function q = proj_IK(p,orient,c)
    % p (m): [x, y, z] desired end effector position expressed in 0th frame
    % orient (rad): the scalar desired end effector orientation angle in
    % the plane. 0 at right horizontal direction and positive in the CCW
    % direction.
    % c (m): constants. [l1, l2, l3, l4]
    % q (rad): joint angles. [t1 t2 t3 t4]
    % Our 4 DOF robot does not have dextrous workspace, and we can
    % only manipulate the orientation in the 2D plane.

    l1 = c(1);
    l2 = c(2);
    l3 = c(3);
    l4 = c(4);
    x = p(1);
    y = p(2);
    z = p(3);

    % From end effector position and orientation, position of the frame4
    % wrt frame1 can be calculated, theta1 is determined.
    [theta, r_proj_1e, z_1e] = cart2pol(x, y, z - l1);
    t1 = theta;
    z4 = z_1e - l4*sin(orient);
    r_proj4 = r_proj_1e - l4*cos(orient);

    % Since frame2 has the same position as frame1, we have a planar
    % problem to solve for theta2 and theta3 given the position at frame4.
    % theta3 is 0 when x3 aligns with x2, and positive CW
    r4_squared = r_proj4.^2 + z4.^2;
    c3 = (r4_squared - l2^2 - l3^2)/(2*l2*l3);
    if 1 < c3^2
        t3 = -100;
        %disp('out of bounds')
    else
        s3 = sqrt(1 - c3.^2);  % Only choose positive theta3
        t3 = atan2(s3, c3);
    end

    % theta2 = alpha + beta (if theta2 is 0 at horizontal and positive CCW)
    alpha = atan2(z4, r_proj4);
    c_beta = (r4_squared + l2^2 - l3^2)/(2*sqrt(r4_squared)*l2);
    if 1 < c_beta^2
        t2 = -100;
        t4 = -100;
        %disp('out of bounds')
    else
        s_beta = sqrt(1 - c_beta.^2);  % Only keep the positive solution
        beta = atan2(s_beta, c_beta);
        
        % Our theta2 is 0 at vertical and positive CCW, therefore -90deg)
        t2 = alpha + beta - pi/2;
        
        % t4 is 0 when x4 aligns with x3, and positive CCW
        % t2 + pi/2 - t3 + t4 = orient
        t4 = orient - t2 - pi/2 + t3;
    end
   
    q = [t1, t2, t3, t4];
    q(q ~= -100) = wrapToPi(q(q ~= -100));

end
