function theta = lin_para(theta_ddot, t0, tf, theta_0, theta_f, t)
    
    theta_h = (theta_f + theta_0)/2;
    th = (t0 + tf)/2;
    
    tb = th - (sqrt((theta_ddot^2)*(th^2) - theta_ddot*(theta_f - theta_0)) / theta_ddot);
    theta_b = theta_0 + 0.5*theta_ddot*(tb^2);
    slope = (theta_h-theta_b) / (th-tb);
    
    theta = zeros(1,length(t));

    for i = 1:length(t)
        if t(i) < tb
            theta(i) = theta_0 + 0.5*theta_ddot*(t(i)^2);
        elseif (t(i) > tb) && (t(i) < (tf-tb))
            theta(i) = theta_b + slope*(t(i)-tb);
        else
            theta(i) = theta_f  - 0.5*theta_ddot*((tf-t(i))^2);
        end
    end

end