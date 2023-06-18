% Function to calculate the transformation matrix given the screw-axis and
% amount of rotation
function out_mat = trans_mat(screw_axis, theta)
    omega_skew = skew_mat(screw_axis(1:3));
    
    out_mat = [eye(3) + sind(theta)*omega_skew + (1 - cosd(theta))*(omega_skew^2), (eye(3)*deg2rad(theta) + (1 - cosd(theta))*omega_skew + (deg2rad(theta) - sind(theta))*(omega_skew^2))*screw_axis(4:6);
               zeros(1, 3), 1];
end