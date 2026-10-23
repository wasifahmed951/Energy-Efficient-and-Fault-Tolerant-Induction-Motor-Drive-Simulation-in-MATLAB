function ast = agv_init(P)
%AGV_INIT  Initial state of the AGV mission (at station A, loaded, at rest).
ast.t      = 0;
ast.pose   = zeros(3,1);     % [x; y; heading]
ast.v      = 0;              % actual forward speed [m/s]
ast.om     = 0;              % actual yaw rate [rad/s]
ast.v_cmd  = 0;              % commanded speed after rate limit [m/s]
ast.s      = 0;              % progress along the path [m]
ast.phase  = 1;              % 1 = to station B, 2 = dwell at B, 3 = back to A, 4 = done
ast.dwell  = 0;
ast.m_tot  = P.agv.m_robot + P.agv.m_payload;
ast.E_J    = 0;              % battery energy used [J]
ast.e_cross = 0;             % cross-track error [m]
ast.e_sum  = 0;              % integral of |cross-track error|
ast.e_max  = 0;
ast.limp   = 0;              % limp-home mode (speed limited)
ast.stop   = 0;              % safe stop (mission aborted)
ast.t_det  = -1;             % time a fault was first reported
ast.s_det  = 0;              % path position at that time
ast.d_stop = 0;              % distance travelled after the fault report
ast.done   = 0;
end
