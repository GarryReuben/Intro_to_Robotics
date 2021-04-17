function[dydx] = mydiffcentral(x,y)

%find length of x
n = length(x);

%find dydx for each point for i = 2:n-1

dydx = zeros(1,n-1);
for i = 2:n-1
    dydx(i) = (y(i+1) - y(i-1)) / (x(i+1) - x(i-1));
end

%Can't calculate derivatives for first and last points, so compute them with forward and backward schemes
dydx(1) = (y(2) - y(1)) / (x(2) - x(1));
dydx(n) = (y(n) - y(n-1)) / (x(n) - x(n-1));

end