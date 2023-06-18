clearvars;
close all;
clc;


%% Defining the tree_robot in code

% Adjacency matrix for the tree
parent = {'J1', 'J2', 'J3', 'J4', 'J4', 'J6', 'J7', 'J8', 'J5', 'J8', 'J9'};
child =  {'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'J8', 'J9', 'E1', 'E2', 'E3'};

% Creating the tree-object
tree = graph(parent, child);

% Naming different links of the robot
tree.Edges.Names = {'L1'; 'L2'; 'L3'; 'L4'; 'L6'; 'L7'; 'L8'; 'L10'; 'L5'; 'L9'; 'L11'};

% Indexing different nodes of the tree robot
tree.Nodes.index = [1; 2; 3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% Defining the joints
tree.Nodes.jointType = ['R'; 'R'; 'R'; 'R'; 'R'; 'P'; 'R'; 'R'; 'P'; 'N'; 'N'; 'N'];

% Defining the joint screw-axes
screw_axes = cell(12,1);
screw_axes(1) = {[1; 0; 0; 0; 0; 0]};
screw_axes(2) = {[1; 0; 0; 0; 2; 0]};
screw_axes(3) = {[1; 0; 0; 0; 5; 0]};
screw_axes(4) = {[0; 0; 1; 0; 0; 0]};
screw_axes(5) = {[1; 0; 0; 0; 6; -2]};
screw_axes(6) = {[0; 0; 0; 0; 1; 0]};
screw_axes(7) = {[0; 0; 1; -3; 0; 0]};
screw_axes(8) = {[0; 0; 1; 3; 6; 0]};
screw_axes(9) = {[0; 0; 0; 0; 6; 0]};
tree.Nodes.screwAxes = screw_axes;

% Initial Configurations
M1 = [[1,0,0;0,0,-1;0,1,0], [0;4;6]; zeros(1, 3), 1];
M2 = [[1,0,0;0,-1,0;0,0,-1], [3;-3;5]; zeros(1, 3), 1];
M3 = [[1,0,0;0,0,1;0,-1,0], [3;-10;6]; zeros(1, 3), 1];

% Tree visualization for clarity
plot(tree);

%% Initial values and parameter definitions

% The end effector that is considered
end_effector = 'E3';

% Target joint displacements
theta_goal1 = [10, 20, 30, 5, 45, 0, 0, 0, 0];
theta_goal2 = [10, 20, 30, 5, 0, 65, 12, 26, 0];
theta_goal3 = [25, 60, 38, 10, 0, 34, 11, 2, 100];

% Initial guess at joint displacements
theta_init1 = [9, 18, 27, 4, 46, 0, 0, 0, 0];
theta_init2 = [9, 18, 27, 4, 0, 69, 10, 23, 0];
theta_init3 = [20, 55, 40, 12, 0, 30, 9, 4, 94];

% Goal, initial configuration, and initial guess to be chosen
if string(end_effector) == "E1"
    M = M1;
    theta_goal = theta_goal1;
    theta_init = theta_init1;
elseif string(end_effector) == "E2"
    M = M2;
    theta_goal = theta_goal2;
    theta_init = theta_init2;
else
    M = M3;
    theta_goal = theta_goal3;
    theta_init = theta_init3;
end

% Desired transformation matrix
Tsd = trans_mat_manipulator(tree, theta_goal, end_effector)*M;


%% Algorithm to iteratively find the solution of the required IK problem

N = 1000;
damper = 0.01;
theta_curr = theta_init;
theta = [transpose(theta_curr), zeros(length(theta_curr), N)];
for i = 1:N
    % To get the current configuration w.r.t space frame {s}
    curr_trans_mat = trans_mat_manipulator(tree, theta_curr, end_effector)*M;
    
    % To define the current jacobian matrix that comes into picture during
    % motion
    curr_J = jacobian(tree, theta_curr, end_effector);
    
    % The twist required to translate from the current configuration to the
    % desired configuration
    curr_twist = adj_mat(curr_trans_mat)*matLog(pinv(curr_trans_mat)*Tsd);
    
    % Updating the value of the solution theta
    theta_curr = theta_curr + damper*transpose(pinv(curr_J)*curr_twist);
    
    % Storing the history of the plausible solutions of theta
    theta(:,i+1) = transpose(theta_curr);
end


%% Plotting the results

% Iterator
itr =0:N;

figure;
subplot(3, 1, 1);
plot(itr, theta(1,:));
title('IK solution for joint-1');
xlabel('No of iterations');
ylabel('\theta_1(t)');
subplot(3, 1, 2);
plot(itr, theta(2,:));
title('IK solution for joint-2');
xlabel('No of iterations');
ylabel('\theta_2(t)');
subplot(3, 1, 3);
plot(itr, theta(3,:));
title('IK solution for joint-3');
xlabel('No of iterations');
ylabel('\theta_3(t)');

figure;
subplot(3, 1, 1);
plot(itr, theta(4,:));
title('IK solution for joint-4');
xlabel('No of iterations');
ylabel('\theta_4(t)');
subplot(3, 1, 2);
plot(itr, theta(5,:));
title('IK solution for joint-5');
xlabel('No of iterations');
ylabel('\theta_5(t)');
subplot(3, 1, 3);
plot(itr, theta(6,:));
title('IK solution for joint-6');
xlabel('No of iterations');
ylabel('\theta_6(t)');

figure;
subplot(3, 1, 1);
plot(itr, theta(7,:));
title('IK solution for joint-7');
xlabel('No of iterations');
ylabel('\theta_7(t)');
subplot(3, 1, 2);
plot(itr, theta(8,:));
title('IK solution for joint-8');
xlabel('No of iterations');
ylabel('\theta_8(t)');
subplot(3, 1, 3);
plot(itr, theta(9,:));
title('IK solution for joint-9');
xlabel('No of iterations');
ylabel('\theta_9(t)');