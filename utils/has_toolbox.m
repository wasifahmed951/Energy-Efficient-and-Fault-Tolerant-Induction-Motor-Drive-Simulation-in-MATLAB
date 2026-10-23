function ok = has_toolbox(name)
%HAS_TOOLBOX  True if a MATLAB toolbox is installed and licensed.
%   name: 'gads' (Global Optimization), 'pct' (Parallel Computing), 'simulink'
ok = false;
if exist('OCTAVE_VERSION', 'builtin'), return; end
switch lower(name)
    case 'gads',     ok = license('test', 'GADS_Toolbox') && ~isempty(ver('globaloptim'));
    case 'pct',      ok = license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));
    case 'simulink', ok = license('test', 'Simulink') && ~isempty(ver('simulink'));
end
end
