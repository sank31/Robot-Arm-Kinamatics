function [dh_matrix] = Make_DH_Matrix(theta, d, a, alpha)

    % Homogeneous transformation Denavit-Hartenberg matrix
    dh_matrix = [cosd(theta), -sind(theta)*cosd(alpha), sind(theta)*sind(alpha),  a*cosd(theta);
                 sind(theta), cosd(theta)*cosd(alpha),  -cosd(theta)*sind(alpha), a*sind(theta);
                 0,           sind(alpha),              cosd(alpha),              d;
                 0,           0,                        0,                        1];
end

