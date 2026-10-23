function [lb, ub, names] = tuning_bounds(P)
%TUNING_BOUNDS  Search bounds in log10 units (same for GA and PSO).
if P.ctrl.type == 1
    names = {'Kp_w','Ki_w','Kp_i','Ki_i'};
    lb = [-1.5, -1.0, 0.0, 2.0];      % 0.03 N m s/rad, 0.1, 1 V/A, 100 V/(A s)
    ub = [ 1.0,  3.0, 2.3, 4.7];      % 10,             1000, 200,  50000
else
    names = {'Kp_w','Ki_w'};
    lb = [-1.5, -1.0];
    ub = [ 1.0,  3.0];
end
end
