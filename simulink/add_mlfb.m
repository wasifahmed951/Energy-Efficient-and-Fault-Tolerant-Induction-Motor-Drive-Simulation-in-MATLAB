function add_mlfb(blk, pos, code_lines, param_names)
%ADD_MLFB  Add a MATLAB Function block with the given code.
%   param_names: function arguments that are read from the workspace as
%   Simulink parameters (e.g. {'P'}) instead of becoming input ports.
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
if isempty(ch)
    error('add_mlfb: could not find the MATLAB Function chart for %s', blk);
end
ch.Script = strjoin(code_lines(:)', newline);
for i = 1:numel(param_names)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', param_names{i});
    if isempty(d)
        error('add_mlfb: argument %s not found in %s', param_names{i}, blk);
    end
    d.Scope = 'Parameter';
end
end
