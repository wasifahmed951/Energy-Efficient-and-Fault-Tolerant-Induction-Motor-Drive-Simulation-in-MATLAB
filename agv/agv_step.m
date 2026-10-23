function [refL, refR, loga, ast] = agv_step(ast, lpL, lcL, lpR, lcR, P)
%AGV_STEP  One sample of the differential-drive AGV: kinematics, energy,
%   pure-pursuit path following, mission logic and fault supervisor.
%
%   lpL/lpR : plant logs of the left/right drive (23), lcL/lcR : controller logs (16)
%   refL/refR : 6-element reference vectors for the two drives
%               [w_ref; T_load; f_speed; f_cur; f_open; J_extra]
%   loga : 16 logged quantities, see agv_log_names in README
%
%   Simplifications (state them in the thesis): rigid wheels without slip,
%   each motor carries half the vehicle mass as reflected inertia, yaw
%   inertia neglected, battery voltage constant (energy accounting only).
a = P.agv;  Ts = P.ctrl.Ts;  g = 9.81;
pth = agv_path(P);
ast.t = ast.t + Ts;
% ---------------- kinematics from the measured motor speeds ----------------
wwL = lpL(4)/a.G;  wwR = lpR(4)/a.G;                   % wheel speeds [rad/s]
v  = a.r_wheel*(wwR + wwL)/2;
om = a.r_wheel*(wwR - wwL)/a.W_track;
ast.v = v;  ast.om = om;
ast.pose = ast.pose + Ts*[v*cos(ast.pose(3)); v*sin(ast.pose(3)); om];
% battery energy: input power of both drives (regeneration counted as negative)
PinL = lpL(9) + lpL(10) + lpL(11) + lpL(12);
PinR = lpR(9) + lpR(10) + lpR(11) + lpR(12);
ast.E_J = ast.E_J + Ts*(PinL + PinR);
% ---------------- progress and cross-track error ----------------
[ast.s, ast.e_cross] = project_on_path(pth, ast.pose(1:2), ast.s);
ast.e_sum = ast.e_sum + Ts*abs(ast.e_cross);
ast.e_max = max(ast.e_max, abs(ast.e_cross));
% ---------------- fault supervisor ----------------
anyf = max([lcL(11:13); lcR(11:13)]) > 0.5;
open_unrem = (lcL(13) > 0.5 && lcL(14) < 0.5) || (lcR(13) > 0.5 && lcR(14) < 0.5);
if anyf && ast.t_det < 0
    ast.t_det = ast.t;  ast.s_det = ast.s;
end
if anyf, ast.limp = 1; end
if open_unrem, ast.stop = 1; end                     % cannot run on a single phase: stop
% numerical divergence of a drive (NaN/Inf): stop the robot and end the
% mission (done = 2 = aborted), so the run ends instead of carrying NaN to Tend
if ~all(isfinite([lpL(4); lpR(4); PinL; PinR; ast.pose]))
    ast.stop = 1;  ast.done = 2;
end
if ast.t_det >= 0, ast.d_stop = ast.d_stop + Ts*abs(v); end
% ---------------- mission logic ----------------
vmax = a.v_max;  if ast.limp > 0, vmax = min(vmax, a.v_limp); end
if ast.phase == 1, s_goal = pth.sB; else, s_goal = pth.sEnd; end
d_goal = max(s_goal - ast.s, 0);
if ast.phase == 1 && d_goal < 0.05 && abs(v) < 0.02
    ast.phase = 2;  ast.dwell = 0;
elseif ast.phase == 2
    ast.dwell = ast.dwell + Ts;
    if ast.dwell >= a.dwell
        ast.phase = 3;  ast.m_tot = a.m_robot;           % payload dropped at B
    end
elseif ast.phase == 3 && d_goal < 0.05 && abs(v) < 0.02 && ast.done < 1.5
    ast.phase = 4;  ast.done = 1;
end
% ---------------- pure pursuit ----------------
lp = point_on_path(pth, min(ast.s + a.L_look, pth.L));
alpha = atan2(lp(2) - ast.pose(2), lp(1) - ast.pose(1)) - ast.pose(3);
alpha = alpha - 2*pi*round(alpha/(2*pi));
kappa = 2*sin(alpha)/a.L_look;
v_corner = sqrt(a.a_lat/max(abs(kappa), 1e-3));
v_tgt = min([vmax, sqrt(2*0.8*a.a_max*d_goal), v_corner]);
if ast.phase == 2 || ast.phase == 4 || ast.stop > 0 || ast.t < a.t_start, v_tgt = 0; end
dv = min(max(v_tgt - ast.v_cmd, -a.a_max*Ts), a.a_max*Ts);
ast.v_cmd = ast.v_cmd + dv;
om_cmd = kappa*ast.v_cmd;
% wheel -> motor speed references
wL_ref = a.G*(ast.v_cmd - om_cmd*a.W_track/2)/a.r_wheel;
wR_ref = a.G*(ast.v_cmd + om_cmd*a.W_track/2)/a.r_wheel;
% ---------------- loads seen by each motor ----------------
F_roll = a.Crr*ast.m_tot*g/2;                         % per wheel [N]
TL_L = a.r_wheel/(a.G*a.eta_g)*F_roll*tanh(wwL/0.2);
TL_R = a.r_wheel/(a.G*a.eta_g)*F_roll*tanh(wwR/0.2);
J_ex = (a.r_wheel/a.G)^2*ast.m_tot/2;
% ---------------- fault injection (experiment definition) ----------------
fL = zeros(3,1);  fR = zeros(3,1);
if a.fault_type > 0 && ast.t >= a.fault_time
    if a.fault_side == 1, fL(a.fault_type) = 1; else, fR(a.fault_type) = 1; end
end
refL = [wL_ref; TL_L; fL; J_ex];
refR = [wR_ref; TL_R; fR; J_ex];
loga = [ast.pose; v; om; ast.v_cmd; ast.E_J/3600; ast.e_cross; ast.m_tot; ast.phase; ...
        ast.limp; ast.stop; ast.done; ast.s; ast.d_stop; ast.t_det];
end

% =====================================================================
function p = point_on_path(pth, s)
k = 1;
while k < numel(pth.s) - 1 && s > pth.s(k+1), k = k + 1; end
f = (s - pth.s(k))/(pth.s(k+1) - pth.s(k));
p = pth.wp(k,:)' + f*(pth.wp(k+1,:)' - pth.wp(k,:)');
end

function [s_best, e] = project_on_path(pth, xy, s_prev)
% closest point on the path, searched only slightly ahead of the last
% progress value so the start/end overlap of the loop is not confused
s_best = s_prev;  d_best = 1e9;  e = 0;
for k = 1:numel(pth.s) - 1
    p0 = pth.wp(k,:)';  p1 = pth.wp(k+1,:)';
    dv = p1 - p0;  L = norm(dv);
    f = min(max(((xy - p0)'*dv)/L^2, 0), 1);
    sk = pth.s(k) + f*L;
    if sk < s_prev - 0.5 || sk > s_prev + 1.0, continue; end
    q = p0 + f*dv;
    dk = norm(xy - q);
    if dk < d_best
        d_best = dk;  s_best = sk;
        e = (dv(1)*(xy(2) - p0(2)) - dv(2)*(xy(1) - p0(1)))/L;   % signed: left of path > 0
    end
end
s_best = max(s_best, s_prev);
end
