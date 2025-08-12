% bldc_build_dataset_disk.m
% Build an on-disk dataset: one MAT per recording + an index table.

clear; clc;

%% --- CONFIG ---
base_path   = 'C:/Users/akac4/Documents/droneMatlab/';  % contains faultNumber=* folders
out_dir     = fullfile(base_path, 'processed_dataset'); % where MAT files + index go
fs_default  = 20000;   % Hz (change if needed)
NUM_HEADER_LINES = 1;  % set to 0 if your .txt has no header
DOWNSAMPLE  = 1;       % 1 = no downsample, 2 = keep every 2nd sample, etc.
SKIP_IF_NO_SPEED = true;  % skip files without a leading signed speed
VERBOSE = true;

if ~exist(out_dir,'dir'), mkdir(out_dir); end

%% --- DISCOVER FAULT FOLDERS ---
dirs = dir(fullfile(base_path, 'faultNumber=*'));
dirs = dirs([dirs.isdir]);

if isempty(dirs)
    error('No folders like "faultNumber=*" found under %s', base_path);
end

% Prepare index (grows per file; memory-light since just metadata)
idx = table('Size',[0 10], ...
    'VariableTypes', {'string','string','double','double','logical','double','double','double','string','string'}, ...
    'VariableNames', {'mat_path','src_path','faultNumber','motor_velocity','transition','fs','num_samples','downsample','folder','filename'});

%% --- LOOP THROUGH FOLDERS/FILES ---
for d = 1:numel(dirs)
    folder_name  = dirs(d).name;                         % e.g. "faultNumber=16" or "... transition"
    fault_tok    = regexp(folder_name,'faultNumber=(\d+)','tokens','once');
    if isempty(fault_tok)
        if VERBOSE, warning('Skip folder (no faultNumber=): %s', folder_name); end
        continue;
    end
    faultNumber  = str2double(fault_tok{1});
    folder_path  = fullfile(dirs(d).folder, folder_name);
    is_tr_folder = contains(folder_name,'transition','ignorecase',true);

    files = dir(fullfile(folder_path, '*.txt'));
    if isempty(files)
        if VERBOSE, fprintf('No .txt files in %s\n', folder_path); end
        continue;
    end

    if VERBOSE
        fprintf('\n== Folder: %s (fault %d%s) ==\n', folder_path, faultNumber, ternary(is_tr_folder,' transition',''));
    end

    for f = 1:numel(files)
        fname = files(f).name;

        % skip obvious notes
        if contains(fname,'explain', 'ignorecase',true) || contains(fname,'important', 'ignorecase',true)
            if VERBOSE, fprintf('  Skip note file: %s\n', fname); end
            continue;
        end

        % Extract speed (first signed integer at filename start)
        tok = regexp(fname, '^(-?\d+)\s', 'tokens', 'once');
        if isempty(tok)
            if SKIP_IF_NO_SPEED
                if VERBOSE, warning('  Skip (no speed at start): %s', fname); end
                continue;
            else
                motor_velocity = NaN;
            end
        else
            motor_velocity = str2double(tok{1});
        end

        src_path = fullfile(files(f).folder, fname);
        if VERBOSE, fprintf('  Loading: %s\n', src_path); end

        % Load numeric matrix
        try
            raw = readmatrix(src_path, 'NumHeaderLines', NUM_HEADER_LINES);
        catch ME
            warning('  Failed to read %s (%s). Skipping.', src_path, ME.message);
            continue;
        end

        % Downsample (optional)
        if DOWNSAMPLE > 1
            raw = raw(1:DOWNSAMPLE:end, :);
        end

        % Build signal struct (small + consistent)
        signal = struct();
        signal.data       = raw;                           % numeric matrix
        signal.fs         = fs_default / DOWNSAMPLE;       % adjust FS if downsampled
        signal.num_samples= size(raw,1);
        signal.num_cols   = size(raw,2);
        signal.meta = struct( ...
            'faultNumber',    faultNumber, ...
            'motor_velocity', motor_velocity, ...
            'transition',     is_tr_folder || contains(fname,'transition','ignorecase',true), ...
            'filename',       fname, ...
            'src_path',       src_path ...
        );

        % Save ONE MAT per file with a readable name
        tag = sprintf('f%d_v%+d%s', faultNumber, motor_velocity, ternary(signal.meta.transition,'_tr',''));
        mat_name = sprintf('%s.mat', sanitize_filename(tag));
        mat_path = fullfile(out_dir, mat_name);

        try
            save(mat_path, 'signal', '-v7.3');  % v7.3 = HDF5 (handles big arrays)
        catch ME
            warning('  Failed to save %s (%s). Skipping.', mat_path, ME.message);
            continue;
        end

        % Append index row
        idx = [idx; {
            string(mat_path), string(src_path), faultNumber, motor_velocity, signal.meta.transition, ...
            signal.fs, double(signal.num_samples), double(DOWNSAMPLE), string(folder_name), string(fname)
        }];
    end
end

% Save index table (.mat + .csv)
idx_path_mat = fullfile(out_dir, 'dataset_index.mat');
idx_path_csv = fullfile(out_dir, 'dataset_index.csv');
save(idx_path_mat, 'idx', '-v7.3');
writetable(idx, idx_path_csv);

fprintf('\nDone. Saved %d recordings to:\n  %s\nIndex written to:\n  %s\n  %s\n', height(idx), out_dir, idx_path_mat, idx_path_csv);

%% --- helpers ---
function s = ternary(cond, a, b), if cond, s=a; else, s=b; end, end
function nm = sanitize_filename(s)
    nm = regexprep(s, '[^\w\-\+\.]', '_');  % keep letters, digits, _, -, +, .
end

