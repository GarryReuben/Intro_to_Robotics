function coeffs = cubic_poly(t0, t1, x_0, x_1, x_dot_0, x_dot_1, x_ddot_0, x_ddot_1)
    % Function to calculate cubic polynomial (fn of time) between 2 points
    % given coordinates and slopes (i.e. velocity) of start and end
    coeffs = zeros(1,6);
    
    A = [ t0^5    t0^4    t0^3  t0^2 t0 1;
          t1^5    t1^4    t1^3  t1^2 t1 1;
         5*t0^4  4*t0^3  3*t0^2 2*t0 1  0;
         5*t1^4  4*t1^3  3*t1^2 2*t1 1  0;
         20*t0^3 12*t0^2  6*t0   2   0  0;
         20*t1^3 12*t1^2  6*t1   2   0  0];
    
    % create matrix of start and end coordinates and slopes
    y = [x_0 x_1 x_dot_0 x_dot_1 x_ddot_0 x_ddot_1]';
        
    coeffs = transpose(A\y);
    
end