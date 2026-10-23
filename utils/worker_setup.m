function worker_setup(sdi_only)
%WORKER_SETUP  Give each parallel worker its own Simulink cache and code
%   generation folder. Without this, workers running sim() at the same time
%   write the same .slxc cache file, which gives the "Unexpected end-of-file
%   while reading compressed data" warnings and can crash MATLAB.
%   (parsim does this by itself; GA/PSO with UseParallel needs it.)
%
%   16 Oct 2026: also stop the Simulation Data Inspector from keeping every
%   run. GA/PSO and the sweeps run thousands of simulations; the stored runs
%   filled the Temp database ("database ... is full"), sim() then failed with
%   "Cannot create variable 'log_plant'", and a worker aborted.
%
%   worker_setup(true) only sets the path and the Data Inspector limit; this
%   is the parsim SetupFcn (parsim gives each worker its own cache folder).
if nargin < 1, sdi_only = false; end
setup_project();
sdi_limit();                       % client and workers
if sdi_only, return; end
t = getCurrentTask();
if isempty(t), return; end
d = fullfile(tempdir, sprintf('imdrive_worker%d', t.ID));
if ~exist(d, 'dir'), mkdir(d); end
Simulink.fileGenControl('set', 'CacheFolder', d, 'CodeGenFolder', d, 'createDir', true);
load_system('IM_DRIVE');
end

function sdi_limit()
% Keep at most one finished run in the Data Inspector and clear what is there.
try
    Simulink.sdi.setAutoArchiveMode(true);    % finished runs move to the archive...
catch
end
try
    Simulink.sdi.setArchiveRunLimit(0);       % ...and the archive keeps none
catch
    try, Simulink.sdi.setArchiveRunLimit(1); catch, end
end
try
    Simulink.sdi.clear;
catch
end
end
