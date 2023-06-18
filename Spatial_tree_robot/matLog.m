function [out_mat] = matLog(trans_matrix)
    theta = acosd((trace(trans_matrix) - 2)/2);
    
    angvel_skew = (1/(2*sind(theta)))*(trans_matrix(1:3, 1:3) - transpose(trans_matrix(1:3, 1:3)));
    
    vel = (eye(3)/deg2rad(theta) - angvel_skew/2 +(1/deg2rad(theta) - 0.5*cotd(theta/2))*(angvel_skew^2))*trans_matrix(1:3,4);
    
    out_mat = [angvel_skew(3,2); angvel_skew(1,3); angvel_skew(2,1); vel];
end