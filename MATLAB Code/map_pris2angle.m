function theta5 = map_pris2angle(pris_d, c)
    max_delta_d = c(5);
    % [eq] d = l/2*cos(theta) + l/2*cos(theta) = lcos(theta)
    % [eq] theta_offset = 30deg
    % [eq] theta5 = theta + theta_offset
    theta5 = -1*acos(1 - pris_d/max_delta_d);
    % theta5 = theta5 + pi/6;
    theta5 = clip(theta5, -pi/4-pi/6, 0);
end