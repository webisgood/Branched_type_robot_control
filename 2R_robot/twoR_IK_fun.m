function [theta1, theta2] = twoR_IK_fun(l1, l2, theta_i, pos_d, N)
    
    theta_curr = theta_i;
    
    for i = 1:N
        J_curr = [-l1*sind(theta_curr(1))-l2*sind(theta_curr(1)+theta_curr(2)), -l2*sind(theta_curr(1)+theta_curr(2));
                  l1*cosd(theta_curr(1))+l2*cosd(theta_curr(1)+theta_curr(2)), l2*cosd(theta_curr(1)+theta_curr(2))];
        pos_curr = [l1*cosd(theta_curr(1))+l2*cosd(theta_curr(1)+theta_curr(2)); l1*sind(theta_curr(1))+l2*sind(theta_curr(1)+theta_curr(2))];
    
        theta_curr = theta_curr - pinv(J_curr)*(pos_curr - pos_d);
    end
    
    theta1 = theta_curr(1);
    theta2 = theta_curr(2);
end

