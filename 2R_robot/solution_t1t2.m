% function to find initial values of theta given the position
function [theta1, theta2] = solution_t1t2(l1, l2, x, y, tp1, tp2)
    syms t1 t2;
    
    eqns = [l1*cos(t1) + l2*cos(t1 + t2) == x, l1*sin(t1) + l2*sin(t1 + t2) == y];
    
    [ts1, ts2] = solve(eqns, [t1, t2]);
    
    ts1 = rad2deg(double(ts1));
    ts2 = rad2deg(double(ts2));
    
    v = sum(abs([ts1 - tp1, ts2 - tp2]), 2);
    
    [val, ind] = min(v);
    
    theta1 = ts1(ind);
    theta2 = ts2(ind);
end