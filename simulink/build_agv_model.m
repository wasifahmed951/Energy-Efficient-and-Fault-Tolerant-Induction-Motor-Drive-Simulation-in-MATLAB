function build_agv_model(mdl)
%BUILD_AGV_MODEL  Create AGV_MISSION.slx: differential-drive AGV with two
%   induction-motor wheel drives (left and right), each a Controller +
%   Plant pair as in IM_DRIVE, and an AGV block (kinematics, pure pursuit,
%   mission, fault supervisor). The simulation stops when the mission ends.
if nargin < 1, mdl = 'AGV_MISSION'; end
here = fileparts(mfilename('fullpath'));
file = fullfile(here, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(file, 'file'), delete(file); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'P.ctrl.Ts', 'StopTime', 'P.agv.Tend', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'SaveState', 'off', ...
    'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');

add_mlfb([mdl '/AGV'], [40 160 200 300], { ...
    'function [refL, refR, loga] = agv(lpL, lcL, lpR, lcR, P)'
    '%#codegen'
    '% Robot kinematics, pure pursuit, mission and fault supervisor (agv/agv_step.m)'
    'persistent ast'
    'if isempty(ast)'
    '    ast = agv_init(P);'
    'end'
    '[refL, refR, loga, ast] = agv_step(ast, lpL, lcL, lpR, lcR, P);'
    }, {'P'});

sides = {'L', 'R'};
y0 = [60, 320];
for i = 1:2
    sd = sides{i};  y = y0(i);
    add_mlfb([mdl '/Controller' sd], [320 y 460 y+90], { ...
        sprintf('function [u, logc] = controller%s(meas, ref, P)', sd)
        '%#codegen'
        'persistent st'
        'if isempty(st)'
        '    st = ctrl_init(P);'
        'end'
        '[u, logc, st] = ctrl_step(st, meas, ref, P);'
        }, {'P'});
    add_mlfb([mdl '/Plant' sd], [540 y 680 y+90], { ...
        sprintf('function [meas, logp] = plant%s(u, ref, P)', sd)
        '%#codegen'
        'persistent pst'
        'if isempty(pst)'
        '    pst = plant_init(P);'
        'end'
        '[meas, logp, pst] = plant_step(pst, u, ref, P);'
        }, {'P'});
    add_block('simulink/Discrete/Unit Delay', [mdl '/Meas delay' sd], ...
        'InitialCondition', 'zeros(4,1)', 'SampleTime', 'P.ctrl.Ts', ...
        'Position', [600 y-40 640 y-10], 'Orientation', 'left');
    add_block('simulink/Discrete/Unit Delay', [mdl '/Plant log delay' sd], ...
        'InitialCondition', 'zeros(23,1)', 'SampleTime', 'P.ctrl.Ts', ...
        'Position', [760 y+20 800 y+50]);
    add_block('simulink/Discrete/Unit Delay', [mdl '/Ctrl log delay' sd], ...
        'InitialCondition', 'zeros(16,1)', 'SampleTime', 'P.ctrl.Ts', ...
        'Position', [500 y+110 540 y+140]);
    add_block('simulink/Sinks/To Workspace', [mdl '/log_plant' sd], ...
        'VariableName', ['log_plant' sd], 'SaveFormat', 'Structure With Time', ...
        'Decimation', 'P.agv.log_decim', 'MaxDataPoints', 'inf', 'Position', [880 y 960 y+30]);
    add_block('simulink/Sinks/To Workspace', [mdl '/log_ctrl' sd], ...
        'VariableName', ['log_ctrl' sd], 'SaveFormat', 'Structure With Time', ...
        'Decimation', 'P.agv.log_decim', 'MaxDataPoints', 'inf', 'Position', [620 y+150 700 y+180]);

    add_line(mdl, sprintf('AGV/%d', i), ['Controller' sd '/2'], 'autorouting', 'on');
    add_line(mdl, sprintf('AGV/%d', i), ['Plant' sd '/2'], 'autorouting', 'on');
    add_line(mdl, ['Controller' sd '/1'], ['Plant' sd '/1'], 'autorouting', 'on');
    add_line(mdl, ['Plant' sd '/1'], ['Meas delay' sd '/1'], 'autorouting', 'on');
    add_line(mdl, ['Meas delay' sd '/1'], ['Controller' sd '/1'], 'autorouting', 'on');
    add_line(mdl, ['Plant' sd '/2'], ['Plant log delay' sd '/1'], 'autorouting', 'on');
    add_line(mdl, ['Plant' sd '/2'], ['log_plant' sd '/1'], 'autorouting', 'on');
    add_line(mdl, ['Controller' sd '/2'], ['Ctrl log delay' sd '/1'], 'autorouting', 'on');
    add_line(mdl, ['Controller' sd '/2'], ['log_ctrl' sd '/1'], 'autorouting', 'on');
end
% AGV inputs: (lpL, lcL, lpR, lcR) through the delays (breaks the algebraic loop)
add_line(mdl, 'Plant log delayL/1', 'AGV/1', 'autorouting', 'on');
add_line(mdl, 'Ctrl log delayL/1', 'AGV/2', 'autorouting', 'on');
add_line(mdl, 'Plant log delayR/1', 'AGV/3', 'autorouting', 'on');
add_line(mdl, 'Ctrl log delayR/1', 'AGV/4', 'autorouting', 'on');

add_block('simulink/Sinks/To Workspace', [mdl '/log_agv'], ...
    'VariableName', 'log_agv', 'SaveFormat', 'Structure With Time', ...
    'Decimation', 'P.agv.log_decim', 'MaxDataPoints', 'inf', 'Position', [240 420 320 450]);
add_line(mdl, 'AGV/3', 'log_agv/1', 'autorouting', 'on');
% stop 0.5 s AFTER the mission is done (log element 13).
% Stopping on the same step as 'done' lost the done flag: the To Workspace
% blocks log only every P.agv.log_decim-th step, so the final step was
% usually not logged and agv_metrics reported completed = 0. The 0.5 s
% run-on matches the MATLAB engine in run_agv.m.
add_block('simulink/Signal Routing/Selector', [mdl '/Select done'], ...
    'IndexMode', 'One-based', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '13', 'InputPortWidth', '16', 'Position', [240 480 280 510]);
add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/Time since done'], ...
    'gainval', '1.0', 'InitialCondition', '0', 'SampleTime', 'P.ctrl.Ts', ...
    'Position', [310 478 350 512]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/After 0.5 s'], ...
    'relop', '>=', 'const', '0.5', 'Position', [380 480 430 510]);
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop at mission end'], 'Position', [460 480 490 510]);
add_line(mdl, 'AGV/3', 'Select done/1', 'autorouting', 'on');
add_line(mdl, 'Select done/1', 'Time since done/1', 'autorouting', 'on');
add_line(mdl, 'Time since done/1', 'After 0.5 s/1', 'autorouting', 'on');
add_line(mdl, 'After 0.5 s/1', 'Stop at mission end/1', 'autorouting', 'on');
% x-y scope
add_block('simulink/Signal Routing/Selector', [mdl '/Select xy'], ...
    'IndexMode', 'One-based', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '[1 2]', 'InputPortWidth', '16', 'Position', [240 540 280 570]);
add_block('simulink/Sinks/Scope', [mdl '/Scope x y'], 'Position', [320 535 350 575]);
add_line(mdl, 'AGV/3', 'Select xy/1', 'autorouting', 'on');
add_line(mdl, 'Select xy/1', 'Scope x y/1', 'autorouting', 'on');

save_system(mdl, file);
fprintf('Saved %s\n', file);
end
