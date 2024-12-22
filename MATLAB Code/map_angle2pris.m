function pris_delta_d = map_angle2pris(theta5, c)
    max_delta_d = c(5);
    % [eq] (d_max - d) = l/2*cos(theta) + l/2*cos(theta) = d_max*cos(theta)
    pris_delta_d = max_delta_d*(1 - cos(theta5));
end