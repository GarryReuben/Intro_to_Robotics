function links = joints(theta1, theta2, d1, d2, d3, L2, L3, Le, m)
    angle = theta1 + theta2;
    
    % Initialise array of 4x4 joint coordinates, x6 points in time
    links = zeros(3,7,m);
    links(end,2,:) = d1*ones(1,m);
    
    links(:,3:end,1) = [L2*cos(theta1(1)) L2*cos(theta1(1)) L2*cos(theta1(1))+L3*cos(angle(1)) L2*cos(theta1(1))+L3*cos(angle(1)) L2*cos(theta1(1))+(L3+Le)*cos(angle(1));
                        L2*sin(theta1(1)) L2*sin(theta1(1)) L2*sin(theta1(1))+L3*sin(angle(1)) L2*sin(theta1(1))+L3*sin(angle(1)) L2*sin(theta1(1))+(L3+Le)*sin(angle(1));
                        d1 d1+d2 d1+d2 d1+d2+d3(1) d1+d2+d3(1)];
    
    links(:,3:end,end) = [L2*cos(theta1(end)) L2*cos(theta1(end)) L2*cos(theta1(end))+L3*cos(angle(end)) L2*cos(theta1(end))+L3*cos(angle(end)) L2*cos(theta1(end))+(L3+Le)*cos(angle(end));
                          L2*sin(theta1(end)) L2*sin(theta1(end)) L2*sin(theta1(end))+L3*sin(angle(end)) L2*sin(theta1(end))+L3*sin(angle(end)) L2*sin(theta1(end))+(L3+Le)*sin(angle(end));
                          d1 d1+d2 d1+d2 d1+d2+d3(end) d1+d2+d3(end)];
    
    n = floor((length(theta1))/(m-1));
    for i = 1:m-2
        t_plot = n*i;
        links(:,3:end,i+1) = [L2*cos(theta1(t_plot)) L2*cos(theta1(t_plot)) L2*cos(theta1(t_plot))+L3*cos(angle(t_plot)) L2*cos(theta1(t_plot))+L3*cos(angle(t_plot)) L2*cos(theta1(t_plot))+(L3+Le)*cos(angle(t_plot));
                              L2*sin(theta1(t_plot)) L2*sin(theta1(t_plot)) L2*sin(theta1(t_plot))+L3*sin(angle(t_plot)) L2*sin(theta1(t_plot))+L3*sin(angle(t_plot)) L2*sin(theta1(t_plot))+(L3+Le)*sin(angle(t_plot));
                              d1 d1+d2 d1+d2 d1+d2+d3(t_plot) d1+d2+d3(t_plot)];

    end
end