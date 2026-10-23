function pth = agv_path(P)
%AGV_PATH  Warehouse route: rectangle Lx x Ly, start/end at station A (0,0),
%   station B at the far corner (Lx, Ly) where the payload is dropped.
%   (The struct is built in one statement so it is code-generation safe.)
Lx = P.agv.Lx;  Ly = P.agv.Ly;
wp = [0 0; Lx 0; Lx Ly; 0 Ly; 0 0];
d = sqrt(sum(diff(wp).^2, 2));
s = [0; cumsum(d)];                 % arc length at each way point
pth = struct('wp', wp, 's', s, 'L', s(end), 'sB', Lx + Ly, 'sEnd', s(end));
end
