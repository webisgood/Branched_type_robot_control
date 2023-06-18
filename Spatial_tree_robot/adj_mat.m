function out_mat = adj_mat(trans_matrix)
    out_mat = [trans_matrix(1:3,1:3), zeros(3,3);
               skew_mat(trans_matrix(1:3,4))*trans_matrix(1:3,1:3), trans_matrix(1:3,1:3)];
end