% Function to evaluate the configuration matrix of the robot given the
% robot object, joint displacements, and the end-effector to be considered.
function [out_mat] = trans_mat_manipulator(tree_robot, theta, end_effector)
    
    % Initializing the tree-robot's node variables
    joint_name = tree_robot.Nodes.Name;
    screw_axes = tree_robot.Nodes.screwAxes;
    joint_type = tree_robot.Nodes.jointType;
    
    % To convert the literal end-effector to its index in the graph nodes
    % so that we can use the inbuilt 'dfs' function
    pos = 1;
    for i = 1:length(joint_name)
        if(joint_name{i} == end_effector)
            pos = i;
            break;
        end
    end
    
    % The effective manipulator that needs to be considered while
    % calculating the jacobian is given by the dfs result from the
    % end-effector to the root of the tree
    dfsResult = dfsearch(tree_robot, pos);
    path = [pos];
    for i  = 2:length(dfsResult)
        path = [path, dfsResult(i)];
    
        if(dfsResult(i) == 1)
            break;
        end
    end
    
    % Iteratively evaluating the spatial transformation matrix for the
    % chosen end-effector
    out_mat = eye(4);
    for i = flip(2:length(path))
        ind = path(i);
        out_mat = out_mat*trans_mat(screw_axes{ind}, theta(ind));
    end
end

