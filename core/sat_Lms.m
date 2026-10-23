function L = sat_Lms(psi_mag, m)
%SAT_LMS  Secant magnetizing inductance for air-gap flux magnitude psi_mag.
%   Lms = Lm0 / (1 + a*(|psi_m|/psi_k)^b). With m.sat_on = 0 returns Lm_rated.
if m.sat_on > 0
    L = m.Lm0 / (1 + m.sat_a * (abs(psi_mag)/m.psi_k)^m.sat_b);
else
    L = m.Lm_rated;
end
end
