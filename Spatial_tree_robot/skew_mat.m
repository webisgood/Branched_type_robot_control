function out_mat = skew_mat(vec)
    out_mat = [0, -vec(3), vec(2);
               vec(3), 0, -vec(1);
               -vec(2), vec(1), 0];
end