%% RRP manipulator trajectory planning and plotting
% Introduction to Robotics (Imperial MechEng 4th year)
% By Garry Reuben Sutrisno
% created 09-04-2021
%% IMPORTANT: SPECIFY WHICH TRAJECTORY PLANNING METHOD TO PLOT (LINE 12)

%% clear all variables and close windows
clear
close all

%% ########### SPECIFY WHICH TRAJECTORY PLANNING METHOD TO PLOT ###########
method = 2; %(1=Cartesian, 2=Joint,3=J)

%% Manipulator DH parameters
% create variables for DH parameters
d1 = 225;
d2 = -20;
L1 = 0;
L2 = 300;
Le = 25;
L3 = 250;
theta3 = 0;

%% Set initial and final points, A and B
% Since workspace is cylindrical, easier to set in polar coordinates

% Set r, theta (degrees), z for A and B:
r_A = 150;
r_B = 500;
angle_A = 170;
angle_B = 50;
z_A = 180;
z_B = 50;

% check whether A or B are outside limits
if (r_A > 550) || (r_B > 550) || (r_A < 55) || (r_B < 55) || (z_A > 200) || (z_B > 200) || (z_A < 5) || (z_B < 5) 
    disp('Start or end point outside of workspace - please change!')
    return %exit entire script
else

% Transform to Cartesian coordinates
A = [r_A*cos(deg2rad(angle_A)) r_A*sin(deg2rad(angle_A)) z_A];
B = [r_B*cos(deg2rad(angle_B)) r_B*sin(deg2rad(angle_B)) z_B];

%% Specify motion duration
dt = 0.01; % time increments
t0 = 0; % initial time
T = 3; % movement duration
t1 = t0 + T; % final time
t = 0:dt:t1; % define time domain

%% Set straight line trajectory plan (in Cartesian coordinates) from A to B
trajectory_plan = zeros(3,length(t));

grad = (B-A)/(t1-t0); % find gradient in 3D
for i = 1:3 % For each coordinate (x, y, z)
    trajectory_plan(i,:) = A(i) + grad(i)*t; % find straight trajectory
end

% Input x, y, and z coordinates into separate arrays (for plotting later)
x_plan = trajectory_plan(1,:);
y_plan = trajectory_plan(2,:);
z_plan = trajectory_plan(3,:);

% Since d3 is not affected by previous joint angles, can calculate
d3_plan = z_plan - d1 - d2;

%% Trajectory planning (in Cartesian coordinates)
% find function between initial and final point in Cartesian space
% using quintic_poly.m or lin_para.m
% then solve inverse kinematics at each point along the trajectory
% This method guarantees it follows the straight path closely,
% but joint angles might change abruptly!

% Using quintic_poly.m
% Set initial and final end effector velocities and accelarations
vel_0 = [0 0 0];
vel_1 = [0 0 0];
a_0 = [0 0 0];
a_1 = [0 0 0];

% Initialise array of quintic polynomial coefficients; 
coeffs = zeros(3,6); % 3 Cartesian coords, 6 coeffs each

% Initialise array of velocities and accelerations
pos_cart = zeros(3,length(t));
vel_cart = zeros(3,length(t));
a_cart = zeros(3,length(t));

% Fit quintic function (in Cartesian space)
% and calculate trajectory, velocity, and acceleration
for i = 1:3 % for each Cartesian coordinate
    coeffs(i,:) = quintic_poly(t0, t1, A(i), B(i), vel_0(i), vel_1(i), a_0(i), a_1(i));
    pos_cart(i,:) = coeffs(i,1)*t.^5 + coeffs(i,2)*t.^4 + coeffs(i,3)*t.^3 + coeffs(i,4)*t.^2 + coeffs(i,5)*t + coeffs(i,6);
    vel_cart(i,:) = 5*coeffs(i,1)*t.^4 + 4*coeffs(i,2)*t.^3 + 3*coeffs(i,3)*t.^2 + 2*coeffs(i,4)*t + coeffs(i,5);
    a_cart(i,:) = 20*coeffs(i,1)*t.^3 + 12*coeffs(i,2)*t.^2 + 6*coeffs(i,3)*t + 2*coeffs(i,4);
end

% Inverse kinematics: find joint variables all along trajectory
% lots of IK operations!
% In joint space there are  2 solutions
% so there will be 2 possible positions at each Cartesian point;
theta_cart = zeros(3,length(x_plan),2);

theta_cart(:,:,1) = IK(pos_cart(1,:), pos_cart(2,:), pos_cart(3,:), L2, L3, Le, d1, d2); % first solution
theta_cart(:,:,2) = [2*pi 2*pi 0]' - theta_cart(:,:,1); % second solution
slice_theta_cart = squeeze(theta_cart(:,:,1)); % input a solution into array for plotting

%% Trajectory planning (joint space)
% Inverse kinematics: find initial and final joint variables (IK only twice!)
% interpolate between the initial and final joint variables 
% for each joint variable with a quintic polynomial
% since there are 2 solutions each at start and end,
% there will be 4 possible initial/final position combinations!
% This method guarantees smooth joint motion, 
% but end effector path in Cartesian space between points isn't controlled!

% Initialise array of the three joint variables, for 4 possible solutions
theta_joint = zeros(3,length(t),4);
vel_joint = zeros(3,length(t),4);
a_joint = zeros(3,length(t),4);

% calculate the 4 combinations of initial and final joint variables
% initial
theta_joint(:,1,1) = IK(A(1), A(2), A(3), L2, L3, Le, d1, d2);
theta_joint(:,1,2) = theta_joint(:,1,1);
theta_joint(:,1,3) = [2*pi 2*pi 0]' - theta_joint(:,1,1);
theta_joint(:,1,4) = theta_joint(:,1,3);

% final
theta_joint(:,end,1) = IK(B(1), B(2), B(3), L2, L3, Le, d1, d2);
theta_joint(:,end,3) = theta_joint(:,end,1);
theta_joint(:,end,2) = [2*pi 2*pi 0]' - theta_joint(:,end,1);
theta_joint(:,end,4) = theta_joint(:,end,2);

% set initial and final velocities (in Cartesian coordinates)
vel_joint_0 = [0,0,0];
vel_joint_1 = [0,0,0];
a_joint_0 = [0,0,0];
a_joint_1 = [0,0,0];

% initialise coeffs array (3 joint variables * 6 coeffs * 4 possible initial/final angle combinations)
coeffs_joint = zeros(3,6,4);

% Fit quintic function (in joint space)
% and calculate trajectory, velocity, and accelerationfor 
for i = 1:4 % for each combination of start and end joint variables
    for j = 1:3 % for each of the 3 joint variables
        coeffs_joint(j,:,i) = quintic_poly(t0, t1, theta_joint(j,1,i), theta_joint(j,end,i), vel_joint_0(j), vel_joint_1(j), a_joint_0(j), a_joint_1(j));
        theta_joint(j,:,i) = coeffs_joint(j,1,i)*t.^5 + coeffs_joint(j,2,i)*t.^4 + coeffs_joint(j,3,i)*t.^3 + coeffs_joint(j,4,i)*t.^2 + coeffs_joint(j,5,i)*t + coeffs_joint(j,6,i);
        vel_joint(j,:,i) = 5*coeffs_joint(j,1,i)*t.^4 + 4*coeffs_joint(j,2,i)*t.^3 + 3*coeffs_joint(j,3,i)*t.^2 + 2*coeffs_joint(j,4,i)*t + coeffs_joint(j,5,i);
        a_joint(j,:,i) = 20*coeffs_joint(j,1,i)*t.^3 + 12*coeffs_joint(j,2,i)*t.^2 + 6*coeffs_joint(j,3,i)*t + 2*coeffs_joint(j,4,i);
    end
end

%% Inverse Jacobian: Use inv(J) and Cartesian velocities to find joint velocities and hence positions
% Find velocities in Cartesian coordinates from quintic polynomial
% use inverse Jacobian to transform to joint velocities, 
% and update positions by multiplying by dt
% smooth joint velocities, while also following path,
% but less accurate as it is a numerical approximation (linearisation)

vel_j = zeros(3,length(t));

% Fit quintic function (in Cartesian space)
% and calculate trajectory, velocity, and acceleration
for i = 1:3 % for each Cartesian coordinate
    coeffs(i,:) = quintic_poly(t0, t1, A(i), B(i), vel_0(i), vel_1(i), a_0(i), a_1(i));
    vel_j(i,:) = 5*coeffs(i,1)*t.^4 + 4*coeffs(i,2)*t.^3 + 3*coeffs(i,3)*t.^2 + 2*coeffs(i,4)*t + coeffs(i,5);
end

% find Jacobian at initial point (using Jacobian.m)
J = Jacobian(theta_joint(1,1,1), theta_joint(2,1,1), theta3, L1, L2, L3, Le);

% initialise array of joint variables
theta_j = zeros(3,length(t));
theta_j(:,1) = [theta_joint(1,1,1) theta_joint(2,1,1) theta_joint(3,1,1)]'; % input initial theta values

% initialise array of joint velocities
theta_dot_j = zeros(3,length(t)); 

% For each point along planned trajectory, 
% find joint velocities with inv(J), find joint positions, and update J
for i = 2:length(t)
    % calculate theta_dot from vel_plan
    theta_dot_j(:,i) = J(1:3,1:3) \ [vel_j(1,i); vel_j(2,i); vel_j(3,i)];
    
    % calculate next theta values, and input into theta_j array
    theta_j(:,i) = theta_j(:,i-1) + theta_dot_j(:,i)*dt;
    
    % update J
    J = Jacobian(theta_j(1,i),theta_j(2,i),theta_j(3,i), L1, L2, L3, Le);
end

%% Calculate velocities in Cartesian and joint spaces
% calculate velocities (using mydiffcentral.m); 
% actual joint/Cartesian velocities may not be a smooth function anymore
% depending on trajectory planning method

% specify which solution for each trajectory planning method
n_cart = 1; 
n_joint = 1;

% calculate Cartesian positions using forward kinematics (FK.m)
trajectory_cart = FK(theta_cart(1,:,n_cart), theta_cart(2,:,n_cart), d1, d2, theta_cart(3,:,n_cart), L2, L3, Le);
trajectory_joint = FK(theta_joint(1,:,n_joint), theta_joint(2,:,n_joint), d1, d2, theta_joint(3,:,n_joint), L2, L3, Le);
trajectory_j = FK(theta_j(1,:), theta_j(2,:), d1, d2, theta_j(3,:), L2, L3, Le);

% squeeze into 2D array for plotting (plot3 can't take 3D arrays)
slice_cart = squeeze(trajectory_cart(:,4,:));
slice_joint = squeeze(trajectory_joint(:,4,:));
slice_j = squeeze(trajectory_j(:,4,:));

% initialise velocity and accelaration arrays
theta_dot_cart = zeros(3,length(t));
theta_ddot_cart = zeros(3,length(t));
cart_dot_joint = zeros(3,length(t));
cart_ddot_joint = zeros(3,length(t));
cart_dot_j = zeros(3,length(t));
cart_ddot_j = zeros(3,length(t));
theta_ddot_j = zeros(3,length(t));

for i = 1:3
    % For Cartesian trajectory planning, only Cartesian motion known
    % find joint space velocities and accelarations
    theta_dot_cart(i,:) = mydiffcentral(t,theta_cart(i,:, n_cart));
    theta_ddot_cart(i,:) = mydiffcentral(t,theta_dot_cart(i,:));
    
    % For joint space trajectory planning, only joint motion known
    % find Cartesian velocities and accelarations
    cart_dot_joint(i,:) = mydiffcentral(t,slice_joint(i,:));
    cart_ddot_joint(i,:) = mydiffcentral(t,cart_dot_joint(i,:));
    
    % For Jacobian trajectory planning, actual motion might deviate from plan
    % especially for large dt values (error accumulates!)
    cart_dot_j(i,:) = mydiffcentral(t,slice_j(i,:));
    cart_ddot_j(i,:) = mydiffcentral(t,cart_dot_j(i,:));
    theta_ddot_j(i,:) = mydiffcentral(t,theta_dot_j(i,:));
end

%% Plot trajectory in 3D space for all three methods
% Plot trajectories of the three methods
figure(1)
    if method == 1
    plot3(slice_cart(1,:), slice_cart(2,:), slice_cart(3,:), 'r', 'linewidth', 4)
    hold on
    elseif method == 2
    plot3(slice_joint(1,:), slice_joint(2,:), slice_joint(3,:), 'g', 'linewidth', 3)
    hold on
    else
    plot3(slice_j(1,:), slice_j(2,:), slice_j(3,:), 'b', 'linewidth', 2)
    hold on
    end
% Plot planned trajectory in Cartesian space
    plot3(x_plan, y_plan, z_plan, 'k--')
    hold on
% Plot start and end, label as A and B
    scatter3(A(1), A(2), A(3), 70, 'co', 'filled')
    hold on
    scatter3(B(1), B(2), B(3), 70, 'mo', 'filled')
    hold on
    title('Trajectory in the task space, showing manipulator positions')

% Draw links (using FK to find position of each joint and plotting lines in between)
m = 3;% number of positions to plot
if method == 1
    links = joints(theta_cart(1,:,n_cart), theta_cart(2,:,n_cart), d1, d2, theta_cart(3,:,n_cart), L2, L3, Le, m);
elseif method == 2
    links = joints(theta_joint(1,:,n_cart), theta_joint(2,:,n_cart), d1, d2, theta_joint(3,:,n_cart), L2, L3, Le, m);
else
    links = joints(theta_j(1,:), theta_j(2,:), d1, d2, theta_j(3,:,n_cart), L2, L3, Le, m);
end

for i = 1:m
    slice_links = squeeze(links(:,:,i));
    plot3(slice_links(1,:), slice_links(2,:), slice_links(3,:), 'k-', 'linewidth', 2)
end

xlabel('x');
ylabel('y');
zlabel('z');
axis('equal')
lims = L2+L3;
xlim([-lims lims]) % set axis limits
ylim([-lims lims])
zlim([0 250])
legend('Actual trajectory', 'Planned trajectory', 'A', 'B')

%% Plot Cartesian positions, velocities and accelerations
% Plot x, y, z against t for all three methods
figure(2)
    if method == 1
    % Cartesian trajectory plan (quintic polynomial fit)
    plot(t,slice_cart(1,:), 'r-', 'linewidth', 2)
    hold on
    plot(t,slice_cart(2,:), 'g-', 'linewidth', 2)
    hold on
    plot(t,slice_cart(3,:), 'b-', 'linewidth', 2)
    hold on
    elseif method == 2
    % Joint space trajectory plan
    plot(t,slice_joint(1,:), 'r--', 'linewidth', 2)
    hold on
    plot(t,slice_joint(2,:), 'g--', 'linewidth', 2)
    hold on
    plot(t,slice_joint(3,:), 'b--', 'linewidth', 2) % will follow the cartesian one;z unnafected by revolutes!
    hold on 
    else
    % Inverse Jacobian trajectory plan
    plot(t,slice_j(1,:), 'r:', 'linewidth', 2)
    hold on
    plot(t,slice_j(2,:), 'g:', 'linewidth', 2)
    hold on
    plot(t,slice_j(3,:), 'b:', 'linewidth', 2)
    hold on
    end

title('x, y, and z values vs. time')
xlabel('t (s)')
ylabel('position (m)')
legend('x', 'y', 'z', 'Location', 'southeast')

% Plot cartesian velocities
figure(3)
    if method == 1
    % Cartesian space trajectory planning
    plot(t, vel_cart(1,:), 'r-', 'linewidth', 2)
    hold on
    plot(t, vel_cart(2,:), 'g-', 'linewidth', 2)
    hold on
    plot(t, vel_cart(3,:), 'b-', 'linewidth', 2)
    hold on
    elseif method == 2
    % joint space trajectory planning
    plot(t, cart_dot_joint(1,:), 'r--', 'linewidth', 2)
    hold on
    plot(t, cart_dot_joint(2,:), 'g--', 'linewidth', 2)
    hold on
    plot(t, cart_dot_joint(3,:), 'b--', 'linewidth', 2) % coincides with plan; it's just linear prismatic motion, unaffected by previous joints
    hold on
    else
    % Jacobian trajectory planning
    plot(t, cart_dot_j(1,:), 'r:', 'linewidth', 2)
    hold on
    plot(t, cart_dot_j(2,:), 'g:', 'linewidth', 2)
    hold on
    plot(t, cart_dot_j(3,:), 'b:', 'linewidth', 2) % coincides with plan; it's just linear prismatic motion, unaffected by previous joints
    hold on
    end
title('Cartesian velocities')
name = legend('$\dot{x}$', '$\dot{y}$', '$\dot{z}$');
set(name,'Interpreter','latex')
xlabel('t (s)')
ylabel('Velocity (m/s)')

% Plot Cartesian accelarations
figure(4)
    if method == 1
    % Cartesian trajectory planning
    plot(t, a_cart(1,:), 'r-', 'linewidth', 2)
    hold on
    plot(t, a_cart(2,:), 'g-', 'linewidth', 2)
    hold on
    plot(t, a_cart(3,:), 'b-', 'linewidth', 2)
    hold on
    elseif method == 2
    % joint space trajectory planning
    plot(t, cart_ddot_joint(1,:), 'r--', 'linewidth', 2)
    hold on
    plot(t, cart_ddot_joint(2,:), 'g--', 'linewidth', 2)
    hold on
    plot(t, cart_ddot_joint(3,:), 'b--', 'linewidth', 2) % coincides with plan; it's just linear prismatic motion, unaffected by previous joints
    hold on
    else
    % Jacobian trajectory planning
    plot(t, cart_ddot_j(1,:), 'r:', 'linewidth', 2)
    hold on
    plot(t, cart_ddot_j(2,:), 'g:', 'linewidth', 2)
    hold on
    plot(t, cart_ddot_j(3,:), 'b:', 'linewidth', 2) % coincides with plan; it's just linear prismatic motion, unaffected by previous joints
    hold on
    end
title('Cartesian accelarations')
name = legend('$\ddot{x}$', '$\ddot{y}$', '$\ddot{z}$');
set(name,'Interpreter','latex')
xlabel('t (s)')
ylabel('Acceleration (m/s^2)')


%% Plot joint positions, velocities, and accelarations
% Plot positions
figure(5)
subplot(2,1,1) % angles
    if method == 1
    % Cartesian trajectory planning
    plot(t, theta_cart(1,:, n_cart), 'r-', 'linewidth', 2)
    hold on
    plot(t, theta_cart(2,:, n_cart), 'g-', 'linewidth', 2)
    hold on
    elseif method == 2
    % joint space trajectory planning
    plot(t, theta_joint(1,:,n_joint), 'r--', 'linewidth', 2)
    hold on
    plot(t, theta_joint(2,:,n_joint), 'g--', 'linewidth', 2)
    hold on
    else
    % Jacobian trajectory planning
    plot(t, theta_j(1,:), 'r:', 'linewidth', 2)
    hold on
    plot(t, theta_j(2,:), 'g:', 'linewidth', 2)
    hold on
    end
    name = legend('${\theta}_{1}$', '${\theta}_{2}$');
    set(name,'Interpreter','latex')
title('Joint variables against time')
    xlabel('t (s)')
    ylabel('Positioni (rad)')

subplot(2,1,2) % d3
    if method == 1
    plot(t, theta_cart(3,:, n_cart), 'b-', 'linewidth', 2)
    hold on
    elseif method == 2
    plot(t, theta_joint(3,:,n_joint), 'b--', 'linewidth', 2)
    hold on
    else
    plot(t, theta_j(3,:), 'b:', 'linewidth', 2) % coincides with plan; it's just linear prismatic motion, unaffected by previous joints
    hold on
    end
    name = legend('${d_3}$');
    set(name,'Interpreter','latex')
    xlabel('t (s)')
    ylabel('Position (mm)')

slice_theta_dot_joint = squeeze(vel_joint(:,:,n_joint));
    
% Plot joint velocities
figure(6)
subplot(2,1,1)
    if method == 1
    plot(t, theta_dot_cart(1,:) , 'r-', 'linewidth', 2)
    hold on
    plot(t, theta_dot_cart(2,:) , 'g-', 'linewidth', 2)
    hold on
    elseif method == 2
    plot(t, slice_theta_dot_joint(1,:) , 'r--', 'linewidth', 2)
    hold on
    plot(t, slice_theta_dot_joint(2,:) , 'g--', 'linewidth', 2)
    hold on
    else
    plot(t, theta_dot_j(1,:) , 'r:', 'linewidth', 2)
    hold on
    plot(t, theta_dot_j(2,:) , 'g:', 'linewidth', 2)
    hold on
    end
    name = legend('$\dot{\theta}_{1}$', '$\dot{\theta}_{2}$', 'Location', 'southeast');
    set(name,'Interpreter','latex')
title('Joint velocities against time')
    xlabel('t (s)')
    ylabel('Velocity (rad/s)')

subplot(2,1,2)
    if method == 1
    plot(t, theta_dot_cart(3,:) , 'b-', 'linewidth', 2)
    hold on
    elseif method == 2
    plot(t, slice_theta_dot_joint(3,:) , 'b--', 'linewidth', 2)
    hold on
    else
    plot(t, theta_dot_j(3,:) , 'b:', 'linewidth', 2)
    hold on
    end
    name = legend('$\dot{d_3}$', 'Location', 'southeast');
    set(name,'Interpreter','latex')
    xlabel('t (s)')
    ylabel('Velocity (mm/s)')

slice_theta_ddot_joint = squeeze(a_joint(:,:,n_joint));

% Plot joint accelarations
figure(7)
subplot(2,1,1)
    if method == 1
    plot(t, theta_ddot_cart(1,:) , 'r-', 'linewidth', 2)
    hold on
    plot(t, theta_ddot_cart(2,:) , 'g-', 'linewidth', 2)
    hold on
    elseif method == 2
    plot(t, slice_theta_ddot_joint(1,:) , 'r--', 'linewidth', 2)
    hold on
    plot(t, slice_theta_ddot_joint(2,:) , 'g--', 'linewidth', 2)
    hold on
    else
    plot(t, theta_ddot_j(1,:) , 'r:', 'linewidth', 2)
    hold on
    plot(t, theta_ddot_j(2,:) , 'g:', 'linewidth', 2)
    hold on
    end
    name = legend('$\ddot{\theta}_{1}$', '$\ddot{\theta}_{2}$', 'Location', 'southeast');
    set(name,'Interpreter','latex')
    ylim([-1.5 1.5])
title('Joint accelerations against time')
    xlabel('t (s)')
    ylabel('Acceleration (rad/s^2)')

subplot(2,1,2)
    if method == 1
    plot(t, theta_ddot_cart(3,:) , 'b-', 'linewidth', 2)
    hold on
    elseif method == 2
    plot(t, slice_theta_ddot_joint(3,:) , 'b--', 'linewidth', 2)
    hold on
    else
    plot(t, theta_ddot_j(3,:) , 'b:', 'linewidth', 2)
    hold on
    end
    name = legend('$\ddot{d_3}$', 'Location', 'southeast');
    set(name,'Interpreter','latex')
    xlabel('t (s)')
    ylabel('Acceleration (mm/s^2)')


%% Plot workspace
d = 50; % number of points to plot
r1 = 50; % inner radius
r2 = 550; % outer radius

R = linspace(r1,r2,d); % linspace between r1 and r2
beta = linspace(0, 2*pi, d); % angles (0-360)

% calculate points along inner and outer circle
X1 = [r1*cos(beta);r1*cos(beta)];
X2 = [r2*cos(beta);r2*cos(beta)];
Y1 = [r1*sin(beta);r1*sin(beta)];
Y2 = [r2*sin(beta);r2*sin(beta)];

% calculate z heights
Z12 = [-85*ones(1,d);200*ones(1,d)];

% calculate meshgrid for top and bottom surface (in polar coordinates)
[R,beta] = meshgrid(R,beta);

% calculate cartesian coordinates of top and botto
X = R.*cos(beta);
Y = R.*sin(beta);
Ztop = 200*ones(size(X));
Zbot = -85*ones(size(X));

figure(8)
    % plot inner and outer surface
    surf(X1,Y1,Z12, 'FaceColor', 'g')
    hold on
    surf(X2,Y2,Z12, 'FaceColor', 'g')
    hold on

    % Plot top and bottom surfaces
    surf(X,Y,Ztop, 'FaceColor', 'g');
    hold on
    surf(X,Y,Zbot, 'FaceColor', 'g');
    hold on
    axis('equal')
    xlabel('x')
    ylabel('y')
    zlabel('z')

end
