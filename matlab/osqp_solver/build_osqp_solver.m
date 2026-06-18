%% Code cleaning functions

function clean_osqp_code(target_dir)
    fprintf('Cleaning C code...\t\t\t\t\t\t');
    files_c = dir(fullfile(target_dir, 'src', '**', '*.c'));
    files_h = dir(fullfile(target_dir, 'include', '**', '*.h'));
    files = [files_c; files_h];
    
    for k = 1:length(files)
        clean_file(fullfile(files(k).folder, files(k).name));
    end
    fprintf('[done]\n');
end

function clean_file(filepath)
    fid = fopen(filepath, 'r');
    if fid == -1, return; end
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid);
    end
    fclose(fid);
    
    out_lines = {};
    stack = struct('is_outputting', true, 'is_resolved', true, 'has_been_true', true);
    
    empty_lines_count = 0;
    
    for i = 1:length(lines)
        line = lines{i};
        line_trim = strtrim(line);
        line_norm = regexprep(line_trim, '^#\s+', '#');
        
        is_preprocessor = true;
        
        if startsWith(line_norm, '#ifdef') || startsWith(line_norm, '#ifndef')
            is_ifdef = startsWith(line_norm, '#ifdef');
            macro = regexprep(line_norm, '^#\w+\s+', '');
            macro = strtok(macro);
            
            if ismember(macro, {'PRINTING', 'PROFILING', 'CTRLC', '__cplusplus'})
                is_true = ~is_ifdef;
                stack = push_state(stack, true, is_true);
                
            elseif ismember(macro, {'EMBEDDED', 'DLONG'})
                is_true = is_ifdef;
                stack = push_state(stack, true, is_true);
                
            else
                stack = push_state(stack, false, false);
                if stack(end).is_outputting
                    out_lines{end+1} = line;
                end
            end
            
        elseif startsWith(line_norm, '#if ')
            if contains(line_norm, 'EMBEDDED != 1')
                stack = push_state(stack, true, false);
            elseif contains(line_norm, 'EMBEDDED == 1')
                stack = push_state(stack, true, true);
            else
                stack = push_state(stack, false, false);
                if stack(end).is_outputting
                    out_lines{end+1} = line;
                end
            end
            
        elseif startsWith(line_norm, '#else')
            if stack(end).is_resolved
                parent_outputting = stack(end-1).is_outputting;
                if parent_outputting
                    stack(end).is_outputting = ~stack(end).has_been_true;
                    stack(end).has_been_true = true; 
                end
            else
                if stack(end).is_outputting
                    out_lines{end+1} = line;
                end
            end
            
        elseif startsWith(line_norm, '#endif')
            if ~stack(end).is_resolved
                if stack(end).is_outputting
                    out_lines{end+1} = line;
                end
            end
            if length(stack) > 1
                stack(end) = [];
            end
            
        else
            is_preprocessor = false;
        end
        
        if ~is_preprocessor && stack(end).is_outputting
            if isempty(line_trim)
                empty_lines_count = empty_lines_count + 1;
                if empty_lines_count <= 2
                    out_lines{end+1} = line; 
                end
            else
                empty_lines_count = 0;
                out_lines{end+1} = line;
            end
        end
    end
    
    fid = fopen(filepath, 'w');
    for i = 1:length(out_lines)
        fprintf(fid, '%s\n', out_lines{i});
    end
    fclose(fid);
end

function stack = push_state(stack, is_resolved, is_true)
    parent_outputting = stack(end).is_outputting;
    new_state.is_resolved = is_resolved;
    new_state.has_been_true = is_resolved && is_true;
    
    if parent_outputting && is_resolved
        new_state.is_outputting = is_true;
    elseif parent_outputting && ~is_resolved
        new_state.is_outputting = true;
    else
        new_state.is_outputting = false;
    end
    
    stack(end+1) = new_state;
end

%% OSQP building

p = load_parameters();
mpc_data = design_MPC(p);

fprintf('--- Starting OSQP Build Process ---\n');

% Add OSQP to path
buildDir = fullfile(pwd, 'osqp_solver');
osqp_matlab_path = fullfile(buildDir, 'osqp-0.6.2-matlab-windows64');
addpath(osqp_matlab_path);

% Clean previous MEX files and build folders
clear mex;
sl_refresh_customizations;

fprintf('Cleaning old build directory...\t\t\t\t\t');
c_code_dir = fullfile(buildDir, 'osqp_c_code');
try rmdir(c_code_dir, 's'); catch; end
fprintf('[done]\n');

% OSQP vectors definition
p_hat = [1; 1; 1; 1; 1; 1; 10; 10; 10];
p_ref = repmat([1; 1; 1; 1; 1; 1; 10; 10; 10], 1, p.Np+1);

l = [p_hat' zeros(1,mpc_data.n*p.Np) repmat(p.X_min, 1, p.Np+1) repmat(p.U_min, 1, p.Nc)]';
u = [p_hat' zeros(1,mpc_data.n*p.Np) repmat(p.X_max, 1, p.Np+1) repmat(p.U_max, 1, p.Nc)]';
q = [reshape(-diag(p.Q_mpc) * p_ref(:, 1:p.Np), [], 1); -mpc_data.Q_T* p.Wterminale * p_ref(:, p.Np+1); zeros(mpc_data.m * p.Nc, 1)];

% Solver initialisation
solver = osqp;
solver.setup(mpc_data.P_osqp, q, mpc_data.A_osqp, l, u, 'warm_start', true, 'verbose', false, 'eps_abs', 1e-1, 'eps_rel', 1e-1, 'max_iter', 100, 'check_termination', 15);

% Code generation
solver.codegen(c_code_dir, 'parameters', 'vectors');

% Code cleaning
clean_osqp_code(c_code_dir);

% Parameters manual definitions
config_file_path = fullfile(buildDir, 'config_mpc.h');
fid = fopen(config_file_path, 'wt');
fprintf(fid, '#define NP %d\n', p.Np);
fprintf(fid, '#define NC %d\n', p.Nc);
fprintf(fid, '#define N_STATES %d\n', mpc_data.n);
fprintf(fid, '#define N_INPUTS %d\n', mpc_data.m);
fclose(fid);

% MEX file build
inc_dir = fullfile(c_code_dir, 'include');
src_dir = fullfile(c_code_dir, 'src', 'osqp');
mex_output_file = fullfile(buildDir, 'osqp_build');
osqp_build_c_path = fullfile(buildDir, 'osqp_build.c');
mex(['-I' inc_dir], '-output', mex_output_file, osqp_build_c_path, fullfile(src_dir, '*.c'));

% Workspace cleaning
clear q l u fid inc_dir c_code_dir p_hat p_ref src_dir solver ans buildDir config_file_path mex_output_file osqp_build_c_path;

rmpath(osqp_matlab_path);
clear osqp_matlab_path;

mpc_data = rmfield(mpc_data, {'P_osqp', 'A_osqp'});
fprintf('--- OSQP Build Complete ---\n');
