function theta = IK(x, y, L2, L3, d3)
    % Solve for theta2
    theta2 = acos((x.^2 + y.^2 - L2^2 - L3^2) ./ (2*L2*L3));
    
    % Find theta1
    K1 = L2 + L3*cos(theta2);
    K2= L3*sin(theta2);
    r = sqrt(K1.^2+ K2.^2);
    lambda = atan2(K1, K2);

    theta1 = atan2(y, x) - atan2(K2, K1);
    
    theta = [theta1; theta2; d3];
end