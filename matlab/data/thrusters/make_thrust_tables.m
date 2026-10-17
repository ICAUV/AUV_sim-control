function make_thrust_tables()
% Builds the 14.8 V thrust tables used by auv1Params.m from the raw Blue
% Robotics public performance data in raw/:
%
%   T500_14v8.csv  from  T500-Public-Performance-Data-12-24V-August-2022.xlsx
%   T200_14v8.csv  from  T200-Public-Performance-Data-10-20V-September-2019.xlsx
%
% Method: take PWM (us) and force (kgf) from the "14 V" and "16 V" sheets,
% convert force to N (x 9.80665) and interpolate linearly in voltage to
% 14.8 V at each PWM. Positive force is forward (PWM > 1500) thrust.
%
% The script stops with an error if a sheet does not have the expected layout
% (columns, voltage, PWM grid), rather than guessing at columns.
%
% Usage: run make_thrust_tables (no arguments, any working directory)

thrDir = fileparts(mfilename('fullpath'));
rawDir = fullfile(thrDir, 'raw');
Vtarget = 14.8;                       % Battery voltage (V)
Vlow = 14; Vhigh = 16;                % Sheets bracketing Vtarget (V)
kgf2N = 9.80665;                      % Newton per kilogram-force

jobs = struct( ...
    'name',   {'T500', 'T200'}, ...
    'source', {'T500-Public-Performance-Data-12-24V-August-2022.xlsx', ...
               'T200-Public-Performance-Data-10-20V-September-2019.xlsx'});

for j = 1:numel(jobs)
    src = fullfile(rawDir, jobs(j).source);
    sheets = sheetnames(src);
    [pwmL, FL] = readSheet(src, sheets, Vlow);
    [pwmH, FH] = readSheet(src, sheets, Vhigh);
    if ~isequal(pwmL, pwmH)
        error('make_thrust_tables:grid', '%s: PWM grids of the %g V and %g V sheets differ.', ...
            jobs(j).source, Vlow, Vhigh);
    end

    frac = (Vtarget - Vlow) / (Vhigh - Vlow);
    F14v8 = kgf2N * (FL + frac * (FH - FL));     % N

    fprintf('%s: %d points, PWM %d-%d us, F(1100) = %.2f N, F(1900) = %.2f N\n', ...
        jobs(j).name, numel(pwmL), pwmL(1), pwmL(end), F14v8(1), F14v8(end));

    out = fullfile(thrDir, sprintf('%s_14v8.csv', jobs(j).name));
    fid = fopen(out, 'w');
    closeFile = onCleanup(@() fclose(fid));
    fprintf(fid, '# %s thrust table at %.1f V (bollard thrust, static)\n', jobs(j).name, Vtarget);
    fprintf(fid, '# Source file: raw/%s\n', jobs(j).source);
    fprintf(fid, '# Sheets: "%d V" and "%d V" (columns "PWM (us)" and "Force (Kg f)")\n', Vlow, Vhigh);
    fprintf(fid, '# Method: force x %.5f (kgf to N), then linear interpolation in voltage to %.1f V at each PWM\n', kgf2N, Vtarget);
    fprintf(fid, '# Generated: %s by make_thrust_tables.m\n', char(datetime('now', 'Format', 'yyyy-MM-dd')));
    fprintf(fid, '# Columns: pwm_us, force_N (positive = forward thrust)\n');
    fprintf(fid, 'pwm_us,force_N\n');
    fprintf(fid, '%d,%.6f\n', [pwmL(:), F14v8(:)]');
    clear closeFile
end
end

function [pwm, force_kgf] = readSheet(src, sheets, V)
% Reads one voltage sheet and checks its layout
name = sheets(strcmp(strrep(sheets, ' ', ''), sprintf('%dV', V)));
if numel(name) ~= 1
    error('make_thrust_tables:sheet', '%s: no unique sheet for %g V.', src, V);
end
T = readcell(src, 'Sheet', name{1});
hdr = string(T(1,:));
expected = ["PWM", "RPM", "Current", "Voltage", "Power", "Force", "Efficiency"];
if size(T,2) ~= 7 || ~all(startsWith(hdr, expected))
    error('make_thrust_tables:layout', ...
        '%s, sheet %s: unexpected columns [%s]; expected PWM, RPM, Current, Voltage, Power, Force, Efficiency.', ...
        src, name{1}, strjoin(hdr, ' | '));
end
body = T(2:end, :);
if ~all(cellfun(@isnumeric, body(:,[1 4 6]), 'UniformOutput', true), 'all') || ...
        any(cellfun(@(x) any(isnan(x)), body(:,[1 4 6])), 'all')
    error('make_thrust_tables:data', '%s, sheet %s: non-numeric or missing PWM/voltage/force values.', src, name{1});
end
pwm = cell2mat(body(:,1));
volt = cell2mat(body(:,4));
force_kgf = cell2mat(body(:,6));
if any(abs(volt - V) > 0.5)
    error('make_thrust_tables:voltage', '%s, sheet %s: voltage column is not %g V.', src, name{1}, V);
end
if any(diff(pwm) <= 0) || pwm(1) ~= 1100 || pwm(end) ~= 1900
    error('make_thrust_tables:pwm', '%s, sheet %s: PWM column is not increasing from 1100 to 1900.', src, name{1});
end
end
