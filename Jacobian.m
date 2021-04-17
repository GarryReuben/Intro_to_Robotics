function J = Jacobian(theta1, theta2, theta3, L1, L2, L3, Le)

    %% Jacobian
    J11 = -L2*sin(theta1) - (L3+Le)*sin(theta1+theta2);
    J12 = -(L3+Le)*sin(theta1+theta2);

    J21 = L2*cos(theta1) + (L3+Le)*cos(theta1+theta2);
    J22 = (L3+Le)*cos(theta1+theta2);
    
    J = [J11 J12 0;
         J21 J22 0; 
          0   0  1;
          0   0  0
          0   0  0
          1   1  0];
    
end