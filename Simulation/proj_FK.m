function [xs, ys, zs, T] = proj_FK(c, q)
    % q (rad): joint angles [t1, t2, t3, t4]
    % c (m): constants [l1, l2, l3, l4]
    % [xs, ys, zs](i) (m): frame i-1 position from frame0.
    % T{i} (4x4): transformation matrix that describes frame i-1 wrt frame0
    % T has length(q) + 2 elements.
    l1 = c(1);
    l2 = c(2);
    l3 = c(3);
    l4 = c(4);
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);
    t4 = q(4);
    % DH: [alpha_i-1, a_i-1, d_i, theta_i]
    DH = [0 0 l1 t1;
          pi/2 0 0 t2+pi/2;
          pi l2 0 t3;
          pi l3 0 t4;
          0 l4 0 0];
    
    To = eye(4);
    T{1} = To;
    xs = zeros(1, length(q) + 2);
    ys = zeros(1, length(q) + 2);
    zs = zeros(1, length(q) + 2);
    for i = 1:(length(q)+1)
        T_i = computeT(DH(i, :));
        To = To*T_i;
        xs(i+1) = To(1, 4);
        ys(i+1) = To(2, 4);
        zs(i+1) = To(3, 4);
        T{i+1} = To;
    end

end

function T = computeT(DH)
    % DH: DH parameters [alpha_i-1, a_i-1, d_i, theta_i]
    % T: transformation matrix from frame i-1 to frame i
    alpha = DH(1);
    a = DH(2);
    theta = DH(4);
    d = DH(3);
    T = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
         0 0 0 1];
end