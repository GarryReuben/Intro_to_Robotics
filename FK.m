function T = FK(theta1, theta2, d1, d2, d3, L2, L3, Le)
    angle = theta1 + theta2;
    
    T = zeros(4,4,length(theta1));
    
    T(1,1,:) = cos(angle);
    T(1,2,:) = -sin(angle);
    T(1,4,:) = L2*cos(theta1) + (L3+Le)*cos(angle);
    
    T(2,1,:) = sin(angle);
    T(2,2,:) = cos(angle);
    T(2,4,:) = L2*sin(theta1) + (L3+Le)*sin(angle);
    
    d = d1 + d2 + d3;
    T(3,4,:) = d;
    
    I = ones(1,length(theta1));
    T(3,3,:) = I;
    T(4,4,:) = I;
end