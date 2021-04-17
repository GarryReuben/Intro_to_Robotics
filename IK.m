function theta = IK(x, y, z, L2, L3, Le, d1, d2)
    theta2 = acos((x.^2 + y.^2 - L2^2 - (L3+Le)^2) ./ (2*L2*(L3+Le)));
    theta1 = atan2(y, x) - acos((x.^2 + y.^2 + L2^2 - (L3+Le)^2) ./ (2*L2*sqrt(x.^2 + y.^2)));
    
    theta = [theta1; theta2; z-d1-d2];
end