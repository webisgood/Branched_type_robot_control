clear all;
close all;
clc;

%% Defining constant parameters and initial values

% Defining link lengths
l1 = 3;
l2 = 2;

% Desired joint angle displacements
theta_d = [30; 100];

% Desired position vector
pos = [l1*cosd(theta_d(1))+l2*cosd(theta_d(1)+theta_d(2)); l1*sind(theta_d(1))+l2*sind(theta_d(1)+theta_d(2))];

% Initial guess at solution
theta0 = [15; 15];

% % Iterations
% No of iterations
N = 500;

% Current value of theta
theta_curr = theta0;

% Array of thetas
theta = zeros(2,N+1);
theta(:,1) = theta0;

% Constant to damp the error in each iteration
damper = 0.9;

%% Algorithm to iteratively update the solution

for i = 1:N
    J_curr = [-l1*sind(theta_curr(1))-l2*sind(theta_curr(1)+theta_curr(2)), -l2*sind(theta_curr(1)+theta_curr(2));
              l1*cosd(theta_curr(1))+l2*cosd(theta_curr(1)+theta_curr(2)), l2*cosd(theta_curr(1)+theta_curr(2))];
    pos_curr = [l1*cosd(theta_curr(1))+l2*cosd(theta_curr(1)+theta_curr(2)); l1*sind(theta_curr(1))+l2*sind(theta_curr(1)+theta_curr(2))];
    
    theta_curr = theta_curr - damper*pinv(J_curr)*(pos_curr - pos);
    theta(:,i+1) = theta_curr;
end

%% Results:

subplot(2,1,1);
plot(0:N,30*ones(N+1,1),'--','DisplayName','Expected joint1 angle');
legend
hold on;
plot(0:N,10*ones(N+1,1),'--','DisplayName','Expected joint2 angle');
plot(0:N,theta(1,:),'DisplayName','theta1');
plot(0:N,theta(2,:),'DisplayName','theta2');
title("Inverse Kinematics Solver without using the screw theory");
xlabel("No of iterations");
ylabel("Current value of joint displacements");
legend("\theta_{1d}", "\theta_{2d}", "\theta_1", "\theta_2");