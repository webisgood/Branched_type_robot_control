% position function
function [x,y] = position(l1, l2, t1, t2)
    x = l1*cosd(t1) + l2*cosd(t1 + t2);
    y = l1*sind(t1) + l2*sind(t1 + t2);
end

