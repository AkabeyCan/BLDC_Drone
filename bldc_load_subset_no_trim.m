% bldc_save_full_and_zoom
clear; clc; close all;

%% === Paths ===
out_dir  = 'C:/Users/akac4/Documents/droneMatlab/processed_dataset/';
fig_root = fullfile(out_dir, 'figures_by_speed_fault');
if ~exist(fig_root,'dir'); mkdir(fig_root); end

%% === Load index ===
% expects table 'idx' with: mat_path, faultNumber, motor_velocity, transition, fs, num_samples
load(fullfile(out_dir,'dataset_index.mat'), 'idx');

%% === What to run ===
want_faults     = unique(idx.faultNumber);
want_speeds     = unique(idx.motor_velocity);
want_transition = unique(idx.transition);

% want_faults = [1];           % or any subset
% want_speeds = [4000 -7000];  % pick speeds
% want_transition = [false true];


%% === Zoom policy ===
zoomN_samples              = 500;      % width of the zoom window (samples)
zoom_center_normal_sec     = 30;       % where to center zoom for normal (=0) runs (seconds)
zoom_center_transition_sec = [];       % [] => auto: middle of recording

% Optional: limit how much of each recording to draw on the "full" figure
max_seconds_full = [];                 % [] = full length; e.g., set 60 to cap at 60 s

fprintf('Iterating %d faults × %d speeds × %d transitions\n', ...
    numel(want_faults), numel(want_speeds), numel(want_transition));

for f = 1:numel(want_faults)
for s = 1:numel(want_speeds)
for tr = 1:numel(want_transition)

    % Filter rows for this (fault, speed, transition)
    sel = idx(idx.faultNumber==want_faults(f) & ...
              idx.motor_velocity==want_speeds(s) & ...
              idx.transition==want_transition(tr), :);
    if isempty(sel); continue; end

    % Output folder: (everything goes in the same place)
    subdir = fig_root;
    if ~exist(subdir,'dir'); mkdir(subdir); end

    fprintf('fault=%d speed=%+d transition=%d  -> %d files\n', ...
        want_faults(f), want_speeds(s), want_transition(tr), height(sel));

    % Process each file
    for i = 1:height(sel)
        matfile = sel.mat_path{i};
        if ~isfile(matfile)
            warning('Missing: %s', matfile);
            continue;
        end

        S   = load(matfile, 'signal');   % struct with fields: fs, num_samples, data
        sig = S.signal;

        fs   = sig.fs;
        N    = sig.num_samples;
        data = sig.data;

        % Forward-fill IMU NaNs so lines don’t break
        if size(data,2) >= 9
            data(:,4:6) = fillmissing(data(:,4:6),'previous','EndValues','nearest');
            data(:,7:9) = fillmissing(data(:,7:9),'previous','EndValues','nearest');
        end

        % ------- Build baseName ONCE (used for both full + zoom saves) -------
        [~, stem] = fileparts(matfile);
        baseName = sprintf('fault_%02d_speed_%+d_transition_%d_%s', ...
            want_faults(f), want_speeds(s), want_transition(tr), stem);

        %% -------- FULL PLOT --------
        Nfull = N;
        if ~isempty(max_seconds_full)
            Nfull = min(N, round(max_seconds_full*fs));
        end
        t = (0:Nfull-1).' / fs;

        fig1 = figure('Visible','off','Position',[100 100 1400 800]); % wide layout
        tlo_full = tiledlayout(fig1,3,1,'TileSpacing','compact','Padding','loose');
        title(tlo_full, sprintf('fault=%d | speed=%+d | transition=%d', ...
            want_faults(f), want_speeds(s), want_transition(tr)), ...
            'FontSize', 24, 'FontWeight','bold');

        % currents
        ax1 = nexttile; plot(ax1, t, data(1:Nfull,1:3)); grid(ax1,'on')
        ylabel(ax1,'current value','FontSize',11);
        legend(ax1,'U','V','W','Location','northeastoutside');
        set(ax1,'FontSize',11);

        % accelerometer
        ax2 = nexttile; plot(ax2, t, data(1:Nfull,4:6)); grid(ax2,'on')
        ylabel(ax2,'accelerometer value','FontSize',11);
        legend(ax2,'X','Y','Z','Location','northeastoutside');
        set(ax2,'FontSize',11);

        % gyroscope
        ax3 = nexttile; plot(ax3, t, data(1:Nfull,7:9)); grid(ax3,'on')
        ylabel(ax3,'gyroscope value','FontSize',11);
        xlabel(ax3,'time [s]','FontSize',11);
        legend(ax3,'X','Y','Z','Location','northeastoutside');
        set(ax3,'FontSize',11);

        % save FULL
        pngFull = fullfile(subdir, sprintf('%s_full.png', baseName));
        figFull = fullfile(subdir, sprintf('%s_full.fig', baseName));
        exportgraphics(fig1, pngFull, 'Resolution', 300);
        % savefig(fig1, figFull);          %(saves matlab figure file)
        close(fig1);

    %% -------- SMART ZOOM (samples on x-axis) --------
% % decide the center time (seconds)(transitionlarin ortasindan almak icin)
% if want_transition(tr)                              % short snippet
%     if isempty(zoom_center_transition_sec)
%         center_sec = (N/fs)/2;                      % middle of file
%     else
%         center_sec = zoom_center_transition_sec;
%     end
% else                                                % normal, long recording
%     center_sec = zoom_center_normal_sec;
% end
% 
% % compute window [start_idx, end_idx] and clamp to [1, N]
% center_idx = round(center_sec * fs);

% --- Choose zoom center ---
auto_center_normal = false;   % keep normals at 30 s (set true if you want auto, later)

if want_transition(tr)
    % auto-detect transition index from the signals
    center_idx = detect_transition_idx(data, fs);
    center_sec = center_idx / fs;
else
    if auto_center_normal
        center_idx = detect_transition_idx(data, fs);
        center_sec = center_idx / fs;
    else
        center_sec = zoom_center_normal_sec;
        center_idx = round(center_sec * fs);
    end
end


start_idx  = max(1, center_idx - floor(zoomN_samples/2));
end_idx    = min(N, start_idx + zoomN_samples - 1);
start_idx  = max(1, end_idx - zoomN_samples + 1);   % fix start if we hit end

% data for the zoom figure
zoom_data = data(start_idx:end_idx, :);
zoom_len  = size(zoom_data,1);
x_samples = (1:zoom_len).';

% make the zoom figure + tiled layout
fig2 = figure('Visible','off','Position',[100 100 1400 800]);  % wider figure
tlo  = tiledlayout(fig2,3,1,'TileSpacing','compact','Padding','loose');

% title with full context
ttl = sprintf('fault=%d | speed=%+d | transition=%d  (zoom: %d samples @ ~%.1f s)', ...
    want_faults(f), want_speeds(s), want_transition(tr), zoom_len, center_sec);
title(tlo, ttl, 'FontSize', 24, 'FontWeight','bold');

% currents
ax1 = nexttile;  plot(ax1, x_samples, zoom_data(:,1:3));  grid(ax1,'on')
ylabel(ax1,'current value','FontSize',11);
legend(ax1,'U','V','W','Location','northeastoutside'); set(ax1,'FontSize',11);

% accelerometer
ax2 = nexttile;  plot(ax2, x_samples, zoom_data(:,4:6));  grid(ax2,'on')
ylabel(ax2,'accelerometer value','FontSize',11);
legend(ax2,'X','Y','Z','Location','northeastoutside'); set(ax2,'FontSize',11);

% gyroscope
ax3 = nexttile;  plot(ax3, x_samples, zoom_data(:,7:9));  grid(ax3,'on')
ylabel(ax3,'gyroscope value','FontSize',11);
xlabel(ax3,'sample number','FontSize',11);
legend(ax3,'X','Y','Z','Location','northeastoutside'); set(ax3,'FontSize',11);

% save ZOOM
pngZoom = fullfile(subdir, sprintf('%s_zoom.png', baseName));
figZoom = fullfile(subdir, sprintf('%s_zoom.fig', baseName));
exportgraphics(fig2, pngZoom, 'Resolution', 300);
%savefig(fig2, figZoom);         %(saves matlab figure file)
close(fig2);


    end
end
end
end

fprintf('Done. See: %s\n', fig_root);
function center_idx = detect_transition_idx(data, fs)
%DETECT_TRANSITION_IDX  Heuristic to find the biggest step/abrupt change.
%   Uses currents (cols 1:3) and gyro (cols 7:9), smooths, then finds the
%   max absolute derivative point. Ignores edges a bit.

    % Guard for short/empty input
    if isempty(data) || size(data,1) < 10
        center_idx = max(1, round(size(data,1)/2));
        return;
    end

    % pick channels
    use_curr = data(:,1:3);                    % U,V,W
    use_gyr  = data(:,7:9);                    % gyro X,Y,Z (may include NaNs)

    % fill NaNs (should already be filled earlier, but just in case)
    use_gyr  = fillmissing(use_gyr,'previous','EndValues','nearest');

    % build 1D features
    i_avg  = mean(use_curr, 2);                % average phase current
    g_mag  = vecnorm(use_gyr, 2, 2);           % gyro magnitude

    % smooth a bit (5 ms window)
    w = max(3, round(0.005*fs));               % ~5 ms
    i_s = movmean(i_avg, w, 'omitnan');
    g_s = movmean(g_mag, w, 'omitnan');

    % normalize (z-score) to combine
    i_s = (i_s - mean(i_s)) / max(std(i_s), eps);
    g_s = (g_s - mean(g_s)) / max(std(g_s), eps);

    score = i_s + 0.5*g_s;                     % weight currents more; tune if needed

    % derivative magnitude
    d = abs([0; diff(score)]);

    % ignore edges (first/last 0.1 s)
    edge = max(1, round(0.1*fs));
    d(1:edge) = 0;
    d(end-edge+1:end) = 0;

    % pick the largest change
    [~, center_idx] = max(d);

    % fallback if something went weird
    if isempty(center_idx) || isnan(center_idx) || center_idx < 1
        center_idx = max(1, round(size(data,1)/2));
    end
end
