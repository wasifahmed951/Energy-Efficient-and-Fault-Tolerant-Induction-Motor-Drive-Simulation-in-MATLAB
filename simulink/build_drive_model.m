function build_drive_model(mdl)
%BUILD_DRIVE_MODEL  Create IM_DRIVE.slx: one induction-motor drive (IFOC or DTC).
%
%   build_drive_model            creates/overwrites simulink/IM_DRIVE.slx
%
%   Layout:
%     Scenario (From Workspace: scen.sl_ref) --> Controller --> Plant --+--> log_plant
%                       |                          ^                    |
%                       +--------------------------|----> Plant         |
%                                                  +-- Unit Delay <-----+ (measurements)
%   Controller and Plant are MATLAB Function blocks that call
%   ctrl_step.m and plant_step.m. Which drive runs (IFOC or DTC) is set by
%   P.ctrl.type, so the same model serves both.
%
%   Needs base-workspace (or SimulationInput) variables P and scen.

if nargin < 1, mdl = 'IM_DRIVE'; end
here = fileparts(mfilename('fullpath'));
file = fullfile(here, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(file, 'file'), delete(file); end
new_system(mdl);

set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'P.ctrl.Ts', 'StopTime', 'scen.Tend', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'SaveState', 'off', ...
    'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');

% ---- blocks ----
add_block('simulink/Sources/From Workspace', [mdl '/Scenario'], ...
    'VariableName', 'scen.sl_ref', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'SampleTime', 'P.ctrl.Ts', ...
    'Position', [40 140 140 180]);

add_mlfb([mdl '/Controller'], [260 60 400 160], { ...
    'function [u, logc] = controller(meas, ref, P)'
    '%#codegen'
    '% Drive controller: IFOC or DTC selected by P.ctrl.type (see core/ctrl_step.m)'
    'persistent st'
    'if isempty(st)'
    '    st = ctrl_init(P);'
    'end'
    '[u, logc, st] = ctrl_step(st, meas, ref, P);'
    }, {'P'});

add_mlfb([mdl '/Plant'], [500 120 640 220], { ...
    'function [meas, logp] = plant(u, ref, P)'
    '%#codegen'
    '% Two-level inverter + saturated induction motor (see core/plant_step.m)'
    'persistent pst'
    'if isempty(pst)'
    '    pst = plant_init(P);'
    'end'
    '[meas, logp, pst] = plant_step(pst, u, ref, P);'
    }, {'P'});

add_block('simulink/Discrete/Unit Delay', [mdl '/Meas delay'], ...
    'InitialCondition', 'zeros(4,1)', 'SampleTime', 'P.ctrl.Ts', ...
    'Position', [560 20 600 60], 'Orientation', 'left');

add_block('simulink/Sinks/To Workspace', [mdl '/log_plant'], ...
    'VariableName', 'log_plant', 'SaveFormat', 'Structure With Time', ...
    'Decimation', 'P.sim.log_decim', 'MaxDataPoints', 'inf', 'Position', [760 180 840 210]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_ctrl'], ...
    'VariableName', 'log_ctrl', 'SaveFormat', 'Structure With Time', ...
    'Decimation', 'P.sim.log_decim', 'MaxDataPoints', 'inf', 'Position', [500 260 580 290]);

add_block('simulink/Signal Routing/Selector', [mdl '/Select speed torque'], ...
    'IndexMode', 'One-based', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '[4 5]', 'InputPortWidth', '23', 'Position', [760 250 800 280]);
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [860 245 890 285]);

% ---- lines ----
add_line(mdl, 'Scenario/1', 'Controller/2', 'autorouting', 'on');
add_line(mdl, 'Scenario/1', 'Plant/2', 'autorouting', 'on');
add_line(mdl, 'Controller/1', 'Plant/1', 'autorouting', 'on');
add_line(mdl, 'Plant/1', 'Meas delay/1', 'autorouting', 'on');
add_line(mdl, 'Meas delay/1', 'Controller/1', 'autorouting', 'on');
add_line(mdl, 'Plant/2', 'log_plant/1', 'autorouting', 'on');
add_line(mdl, 'Controller/2', 'log_ctrl/1', 'autorouting', 'on');
add_line(mdl, 'Plant/2', 'Select speed torque/1', 'autorouting', 'on');
add_line(mdl, 'Select speed torque/1', 'Scope/1', 'autorouting', 'on');

save_system(mdl, file);
fprintf('Saved %s\n', file);
end
