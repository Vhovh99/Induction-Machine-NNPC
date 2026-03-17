%% Generate load torque profile for Simulink (per-unit)
% Creates variable:
%   Tvar    -> [time, torque_pu] matrix for "From Workspace" blocks
% Optional helper variables:
%   Tvar_ts -> timeseries version of the same profile
%   Tvar_regions -> table with region timing
%   Tvar_meta -> struct with generation settings
%
% User-tunable knobs (define before running this script):
%   tEnd          : total profile duration [s], default 30 min
%   cycleDuration : duration of one operation cycle [s], default 60
%   dTvar        : profile sample time [s], auto-selected if omitted

if ~exist('tEnd', 'var')
    tEnd = 30 * 60;
end

if ~exist('cycleDuration', 'var')
    cycleDuration = 60;
end

if ~exist('dTvar', 'var')
    if tEnd >= 10*60
        dTvar = 1e-2; % 100 Hz torque profile for long runs
    elseif exist('Tsc', 'var')
        dTvar = max(Tsc, 1e-4);
    else
        dTvar = 1e-4;
    end
end

cycleDuration = min(cycleDuration, tEnd);
t = (0:dTvar:tEnd).';
if t(end) < tEnd
    t = [t; tEnd];
end

x = mod(t, cycleDuration) / cycleDuration; % normalized cycle time in [0, 1]
tau = zeros(size(t));   % torque in p.u.
m = @(a, b) (x >= a) & (x < b);

% Define torque profile with step changes every ~2.4 seconds (0.04 of 60s cycle)
% Each region: [normalized_start, normalized_end, torque_value]
regions = [
    0.00, 0.04, 1.00;
    0.04, 0.08, 0.90;
    0.08, 0.12, 0.50;
    0.12, 0.16, 0.60;
    0.16, 0.20, 0.40;
    0.20, 0.24, 1.40;
    0.24, 0.28, 0.70;
    0.28, 0.32, 1.20;
    0.32, 0.36, 0.30;
    0.36, 0.40, 0.80;
    0.40, 0.44, 1.10;
    0.44, 0.48, 0.55;
    0.48, 0.52, 1.50;
    0.52, 0.56, 0.35;
    0.56, 0.60, 0.95;
    0.60, 0.64, 0.45;
    0.64, 0.68, 1.30;
    0.68, 0.72, 0.65;
    0.72, 0.76, 1.60;
    0.76, 0.80, 0.25;
    0.80, 0.84, 0.75;
    0.84, 0.88, 1.05;
    0.88, 0.92, 0.50;
    0.92, 0.96, 1.25;
    0.96, 1.00, 0.85;
];

% Apply regions to torque profile
for i = 1:size(regions, 1)
    tau(m(regions(i, 1), regions(i, 2))) = regions(i, 3);
end

% Make all torque values negative per requirement
% tau = -tau;

% Cycle-to-cycle amplitude variation for richer training data
cycleIndex = floor(t / cycleDuration);
amp = 1 + 0.05 * sin(2*pi*cycleIndex/17);
tau = tau .* amp;
% Force initial startup period to 0 torque (absolute time 0 to 1.6s)
tau(t <= 1.6) = 0;
% Physical clipping in p.u. for stability (expanded for negative loads)
tau = max(min(tau, 2.0), -2.5);

% Main output for Simulink as timeseries
Tvar = timeseries(tau, t, 'Name', 'Tvar');
setinterpmethod(Tvar, 'zoh');

% Optional matrix representation if needed elsewhere
Tvar_matrix = [t, tau];

% Region reference table (useful when labeling logged datasets)
Tvar_regions = table( ...
    repmat({'Step'}, 25, 1), ...
    regions(:, 1) * cycleDuration, ...
    regions(:, 2) * cycleDuration, ...
    regions(:, 3), ...
    'VariableNames', {'Type', 'tStart_s', 'tEnd_s', 'TorqueValue_pu'});

Tvar_meta = struct( ...
    'tEnd_s', tEnd, ...
    'cycleDuration_s', cycleDuration, ...
    'dTvar_s', dTvar, ...
    'nSamples', numel(t), ...
    'nCycles', ceil(tEnd / cycleDuration));

% Quick diagnostic plot
figure; 
plot(Tvar.time(1:min(10000, numel(Tvar.time))), Tvar.data(1:min(10000, numel(Tvar.time))));
xlabel('Time (s)'); ylabel('Torque (p.u.)');
title(sprintf('First 10000 samples - range [%.3f, %.3f]', ...
    min(Tvar.data(1:min(10000, numel(Tvar.data)))), ...
    max(Tvar.data(1:min(10000, numel(Tvar.data))))));
grid on;
