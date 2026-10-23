function M = agv_metrics(res)
%AGV_METRICS  Robot-level results of one mission (proposal 5.7).
%   AGV log columns: 1-3 x,y,heading | 4 v | 5 yaw rate | 6 v_cmd | 7 energy [Wh]
%   8 cross-track error [m] | 9 mass | 10 phase | 11 limp | 12 stop | 13 done
%   14 path progress [m] | 15 distance after fault report [m] | 16 fault report time [s]
%   completed, mission_time [s], E_Wh, Wh_per_m, runtime_h (estimated from
%   the battery size), e_cross_mean_cm, e_cross_max_cm, t_detect [s],
%   stop_distance_m (distance driven after the fault report), limp, stopped.
la = res.la;  t = res.t;  P = res.P;
pth = agv_path(P);
% done flag (col 13): 1 = mission finished, 2 = aborted (a drive diverged to NaN/Inf)
M.aborted = any(la(:,13) > 1.5);
M.completed = any(la(:,13) > 0.5 & la(:,13) < 1.5);
kd = find(la(:,13) > 0.5, 1);
if isempty(kd), kd = numel(t); end
M.mission_time = t(kd);
M.E_Wh = la(kd, 7);
M.distance_m = la(kd, 14);
M.Wh_per_m = M.E_Wh/max(M.distance_m, 1e-3);
M.runtime_h = P.agv.E_batt_Wh/max(M.E_Wh, 1e-6)*M.mission_time/3600;
moving = la(1:kd,4) > 0.05;
e = abs(la(1:kd,8));
M.e_cross_mean_cm = mean(e(moving))*100;
M.e_cross_max_cm = max(e)*100;
M.limp = any(la(:,11) > 0.5);
M.stopped = any(la(:,12) > 0.5);
td = la(end,16);
M.t_detect = NaN;  if td >= 0, M.t_detect = td; end
M.stop_distance_m = NaN;
if M.stopped, M.stop_distance_m = la(end,15); end
M.path_length_m = pth.L;
end
