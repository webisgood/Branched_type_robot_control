clear all;
close all;
clc;

%% Defining some constant parameters and initial values

% Defining link lengths
l1 = 3;
l2 = 2;

% Time vector
space = 0.01;
t = 0:space:20;


%% Desired position and velocity vectors

% Straight-line trajectory
% Straight line from (2, 2) to (3, 3)
pos_d_line = [2 + (1/t(length(t)))*t; 2 + (1/t(length(t)))*t];

% Square trajectory
% (2, 2) -> (3, 2) -> (3, 3) -> (2, 3) -> (2, 2)
step = 1/t(floor(length(t)/4));
pos_d_square = zeros(2,length(t));
for i = 1:length(t)
    if(i <= floor(length(t)/4))
        pos_d_square(:,i) = [2 + step*t(i); 2];
    elseif(i <= floor(length(t)/2))
        pos_d_square(:,i) = [3; 2 + step*(t(i) - t(floor(length(t)/4)))];
    elseif(i <= floor(3*length(t)/4))
        pos_d_square(:,i) = [3 - step*(t(i) - t(floor(length(t)/2))); 3];
    else
        pos_d_square(:,i) = [2; 3 - step*(t(i) - t(floor(3*length(t)/4)))];
    end
end

% Circle trajectory
% Traces a circle centred at (2, 2) of radius = 1
pos_d_circle = [2.5 + (1/sqrt(2))*cos(2*pi*(1/20)*t); 2.5 + (1/sqrt(2))*sin(2*pi*(1/20)*t)];


%% Calculating the joint velocities required to move the end-effector in specific trajectory

% Computation time (no of iterations)
ct = 10;

% Initial theta estimatesS
tg1 = 15;
tg2 = 15;

% Circular trajectory
[t1_c, t2_c] = solution_t1t2(l1, l2, pos_d_circle(1,1), pos_d_circle(2,1), tg1, tg2);
theta_i_c = [t1_c; t2_c];

theta_a_circle = [theta_i_c, zeros(2, length(t)-1)];
pos_a_circle = [pos_d_circle(:,1), zeros(2,length(t)-1)];
for i = 2:length(t)
    [t1_c, t2_c] = twoR_IK_fun(l1, l2, theta_a_circle(:,i-1), pos_d_circle(:,i), ct);
    theta_a_circle(:,i) = [t1_c; t2_c];
    
    [p1_c,p2_c] = position(l1, l2, theta_a_circle(1,i), theta_a_circle(2,i));
    pos_a_circle(:,i) = [p1_c; p2_c];
end

% Square trajectory
[t1_sq, t2_sq] = solution_t1t2(l1, l2, pos_d_square(1), pos_d_square(2), tg1, tg2);
theta_i_sq = [t1_sq; t2_sq];

theta_a_square = [theta_i_sq, zeros(2, length(t)-1)];
pos_a_square = [pos_d_square(:,1), zeros(2,length(t)-1)];
for i = 2:length(t)
    [t1_sq, t2_sq] = twoR_IK_fun(l1, l2, theta_a_square(:,i-1), pos_d_square(:,i), ct);
    theta_a_square(:,i) = [t1_sq; t2_sq];
    
    [p1_sq,p2_sq] = position(l1, l2, theta_a_square(1,i), theta_a_square(2,i));
    pos_a_square(:,i) = [p1_sq; p2_sq];
end

% Straight-line trajectory
[t1_l, t2_l] = solution_t1t2(l1, l2, pos_d_line(1), pos_d_line(2), tg1, tg2);
theta_i_l = [t1_l; t2_l];

theta_a_line = [theta_i_l, zeros(2, length(t)-1)];
pos_a_line = [pos_d_line(:,1), zeros(2,length(t)-1)];
for i = 2:length(t)
    [t1_l, t2_l] = twoR_IK_fun(l1, l2, theta_a_line(:,i-1), pos_d_line(:,i), ct);
    theta_a_line(:,i) = [t1_l; t2_l];
    
    [p1_l,p2_l] = position(l1, l2, theta_a_line(1,i), theta_a_line(2,i));
    pos_a_line(:,i) = [p1_l; p2_l];
end

%% Plotting the results

% Circular Trajectory
figure;
subplot(2,1,1);
plot(t, theta_a_circle(1,:));
title('Time history of joint-1 displacements for circular trajectory');
xlabel("time (t)");
ylabel("Joint-1 displacement (\theta_1)");

subplot(2,1,2);
plot(t, theta_a_circle(2,:));
title('Time history of joint-2 displacements for circular trajectory');
xlabel("time (t)");
ylabel("Joint-2 displacement (\theta_2)");

figure;
plot(pos_d_circle(1,:), pos_d_circle(2,:));
title('Circular trajectory');
hold on
plot(pos_a_circle(1,:), pos_a_circle(2,:));
xlabel("x-coordinate in space");
ylabel("y-coordinate in space");
axis equal;

% Straight-line trajectory
figure;
subplot(2,1,1);
plot(t, theta_a_line(1,:));
title('Time history of joint-1 displacements for line trajectory');
xlabel("time (t)");
ylabel("Joint-1 displacement (\theta_1)");

subplot(2,1,2);
plot(t, theta_a_line(2,:));
title('Time history of joint-2 displacements for line trajectory');
xlabel("time (t)");
ylabel("Joint-2 displacement (\theta_2)");

figure;
plot(pos_d_line(1,:), pos_d_line(2,:));
title('Straight-line trajectory');
hold on
plot(pos_a_line(1,:), pos_a_line(2,:));
xlabel("x-coordinate in space");
ylabel("y-coordinate in space");
axis equal;

% Square trajectory
figure;
subplot(2,1,1);
plot(t, theta_a_square(1,:));
title('Time history of joint-1 displacements for square trajectory');
xlabel("time (t)");
ylabel("Joint-1 displacement (\theta_1)");

subplot(2,1,2);
plot(t, theta_a_square(2,:));
title('Time history of joint-2 displacements for square trajectory');
xlabel("time (t)");
ylabel("Joint-2 displacement (\theta_2)");

figure;
plot(pos_d_square(1,:), pos_d_square(2,:));
hold on
plot(pos_a_square(1,:), pos_a_square(2,:));
title('Square trajectory');
xlabel("x-coordinate in space");
ylabel("y-coordinate in space");
axis equal;