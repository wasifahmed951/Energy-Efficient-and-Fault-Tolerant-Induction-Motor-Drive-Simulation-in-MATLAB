%RUN_BLOCK_DEMO  Run the block-diagram drives with rated and loss-minimising flux.
%
%   Run from the 'Electrical Drive' folder:   run_block_demo
%
%   1. builds IFOC_BLOCKS.slx and DTC_BLOCKS.slx if they do not exist yet;
%   2. runs each model twice on the manual test cycle (1000 rpm, light load of
%      5 N m): rated flux (Flux select = 1) and the analytic loss-minimising
%      look-up table (Flux select = 3, the thesis flux_mode 3);
%   3. prints the steady-state input power (mean over the last 0.3 s) and the
%      saving, and plots speed and input power.
%
%   This shows how to change the top-level blocks from a script. By hand you
%   double-click 'Flux select' or 'Load torque (Nm)' and press Run.
%   To check the block models against the thesis script, use compare_block_vs_script.

here = fileparts(mfilename('fullpath'));
addpath(here);
cd(fileparts(here));                          % run from the Electrical Drive folder
setup_project();

models = {'IFOC_BLOCKS', 'DTC_BLOCKS'};
if ~exist(fullfile(here, 'IFOC_BLOCKS.slx'), 'file') || ~exist(fullfile(here, 'DTC_BLOCKS.slx'), 'file')
    build_block_models({'IFOC', 'DTC'});
end

T_END = 1.5;                                   % long enough for the flux to settle after the load step
flux = [1 3];  flux_names = {'rated flux', 'loss-min LUT'};
Pss = zeros(numel(models), 2);
figure('Name', 'Block models: rated vs loss-minimising flux', 'Color', 'w');
for i = 1:numel(models)
    mdl = models{i};
    load_system(fullfile(here, [mdl '.slx']));
    old = {get_param([mdl '/Load torque (Nm)'], 'After'), get_param([mdl '/Flux select'], 'Value'), ...
           get_param([mdl '/Reference source'], 'Value')};
    set_param([mdl '/Reference source'], 'Value', '1');
    set_param([mdl '/Load torque (Nm)'], 'After', '5');
    for fs = 1:2
        set_param([mdl '/Flux select'], 'Value', num2str(flux(fs)));
        fprintf('%s, %s ... ', mdl, flux_names{fs});
        tic;  out = sim(mdl, 'StopTime', num2str(T_END));  fprintf('%.0f s\n', toc);
        L = out.get('bm_log');
        t = L.time;  y = squeeze(L.signals.values);
        if size(y, 1) ~= numel(t), y = y'; end
        Pin = sum(y(:, 7:10), 2);                       % Pel + Pcond + Psw + Pfe
        win = t >= T_END - 0.3;
        Pss(i, fs) = (y(find(win, 1, 'last'), 25:28) - y(find(win, 1), 25:28))*ones(4, 1)/(t(find(win, 1, 'last')) - t(find(win, 1)));
        subplot(2, numel(models), i); hold on; grid on;
        plot(t, y(:, 2)*60/(2*pi), 'DisplayName', flux_names{fs});
        subplot(2, numel(models), numel(models) + i); hold on; grid on;
        plot(t(1:50:end), Pin(1:50:end), 'DisplayName', flux_names{fs});
    end
    subplot(2, numel(models), i);  title(strrep(mdl, '_', ' '));  ylabel('speed (rpm)');  legend('Location', 'southeast');
    subplot(2, numel(models), numel(models) + i);  ylabel('input power (W)');  xlabel('time (s)');  legend;
    set_param([mdl '/Load torque (Nm)'], 'After', old{1});
    set_param([mdl '/Flux select'], 'Value', old{2});
    set_param([mdl '/Reference source'], 'Value', old{3});
    close_system(mdl, 0);
end

fprintf('\nSteady-state input power at 1000 rpm, 5 N m (mean of the last 0.3 s):\n');
fprintf('%-12s %12s %14s %10s\n', 'model', 'rated flux', 'loss-min LUT', 'saving');
for i = 1:numel(models)
    fprintf('%-12s %10.1f W %12.1f W %9.1f %%\n', models{i}, Pss(i,1), Pss(i,2), 100*(Pss(i,1) - Pss(i,2))/Pss(i,1));
end
