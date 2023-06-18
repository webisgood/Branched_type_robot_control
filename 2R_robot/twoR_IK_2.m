%% Defining constant parameters and initial values

% Defining link lengths
l1 = 3;
l2 = 2;

% Initial guess at the solution
theta0 = [15, 15];

% Desired joint displacements
theta_d = [30; 100];

% Desired position & orientation
pos = [l1*cosd(theta_d(1))+l2*cosd(theta_d(1)+theta_d(2)); l1*sind(theta_d(1))+l2*sind(theta_d(1)+theta_d(2))];
orien = theta_d(1)+theta_d(2);

% Desired transformation of end effector w.r.t space frame
Tsd = [cosd(orien), -sind(orien), 0, pos(1);
       sind(orien), cosd(orien), 0, pos(2);
       0, 0, 1, 0;
       0, 0, 0, 1];

% Initial value of theta
theta_curr = theta0;

% Constant to damp the error in each iteration
damper = 0.5;

%% Algorithm to compute the jacobian and finally iteratively converge to the solution

N = 500;
theta = zeros(N+1,2);
theta(1,:) = theta0;
for i = 1:N
    % Current orientation
    orien_curr = theta_curr(1)+theta_curr(2);
    
    % Current position
    pos_curr = [l1*cosd(theta_curr(1))+l2*cosd(theta_curr(1)+theta_curr(2)), l1*sind(theta_curr(1))+l2*sind(theta_curr(1)+theta_curr(2))];
    
    % Current transformation of {b} w.r.t {s}
    curr_Tsb = [cosd(orien_curr), -sind(orien_curr), 0, pos_curr(1);
                sind(orien_curr), cosd(orien_curr), 0, pos_curr(2);
                0, 0, 1, 0;
                0, 0, 0, 1];
    
    % Overall transformation matrix
    trans_mat = pinv(curr_Tsb)*Tsd;
    
    % Angular velocity in the body frame {b}
    curr_angvel_box = (1/(2*sind(orien_curr)))*(trans_mat(1:3, 1:3) - transpose(trans_mat(1:3, 1:3)));

    % Linear velocity in the body frame {b}
    curr_vel = (eye(3)/deg2rad(orien_curr) - curr_angvel_box/2 +(1/deg2rad(orien_curr) - 0.5*cotd(orien_curr/2))*(curr_angvel_box^2))*trans_mat(1:3,4);

    % Twist in the body frame {b}
    curr_twist = [curr_angvel_box(3,2); curr_angvel_box(1,3); curr_angvel_box(2,1); curr_vel];
    
    % Screw axes of joints w.r.t body frame {b}
    screw1_curr = [0;0;1;0;l1+l2;0];
    screw2_curr = [0;0;1;0;l2;0];

    % Jacobian computation
    J_b = zeros(6,2);
    
    % Jacobian component for joint-1
    % Skew-symmetric representation of S_omega
    omega_skew_curr = [0,-screw2_curr(3),screw2_curr(2);
                       screw2_curr(3),0,-screw2_curr(1);
                       -screw2_curr(2),screw2_curr(1),0];
    
    % Body transformation matrix due to rotation of joint-2
    trans_mat_curr = inv([eye(3)+sind(theta_curr(2))*omega_skew_curr+(1-cosd(theta_curr(2)))*omega_skew_curr^2, (eye(3)*deg2rad(theta_curr(2))+(1-cosd(theta_curr(2)))*omega_skew_curr+(deg2rad(theta_curr(2)-sind(theta_curr(2))))*omega_skew_curr^2)*screw2_curr(4:6);
        zeros(1,3), 1]);
    
    % Skew-matrix of position vector
    pos_skew_curr = [0,-trans_mat_curr(3,4),trans_mat_curr(2,4);
                    trans_mat_curr(3,4),0,-trans_mat_curr(1,4);
                    -trans_mat_curr(2,4),trans_mat_curr(1,4),0];
    
    % Adjoint Matrix of joint-2 w.r.t body {b}
    adjoint_mat_curr = [trans_mat_curr(1:3,1:3), zeros(3,3);
                        pos_skew_curr*trans_mat_curr(1:3,1:3), trans_mat_curr(1:3,1:3)];
    
    % Jacobian component for joint-1
    J_b(:,1) = adjoint_mat_curr*screw1_curr;
    
    % Jacobian component for joint-2
    J_b(:,2) = screw2_curr;

    theta_curr = theta_curr + damper*transpose(pinv(J_b)*curr_twist);
    theta(i+1,:) = theta_curr;
end


%% Results:

subplot(2,1,2);
plot(0:N,30*ones(N+1,1),'--');
hold on;
plot(0:N,10*ones(N+1,1),'--');
plot(0:N,theta(:,1));
plot(0:N,theta(:,2));
title("Inverse Kinematics Solver using the screw theory");
xlabel("No of iterations");
ylabel("Current value of joint displacements");
legend("\theta_{1d}", "\theta_{2d}", "\theta_1", "\theta_2");
