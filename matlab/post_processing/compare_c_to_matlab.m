function compare_c_to_matlab()
    file1 = '../checker/c_Castle_hex.csv';
    file2 = '../checker/matlab_Castle_hex.csv';
    file_output = 'log_delta.csv';
    
    opts = detectImportOptions(file1); 
    opts.VariableTypes(:) = {'char'}; 
    
    data1_raw = readtable(file1, opts);
    data2_raw = readtable(file2, opts);
    
    if size(data1_raw) ~= size(data2_raw)
        error('Error : The two csv files do not have the same dimensions (%dx%d vs %dx%d).', ...
            size(data1_raw, 1), size(data1_raw, 2), size(data2_raw, 1), size(data2_raw, 2));
    end
    
    num_rows = size(data1_raw, 1);
    num_cols = size(data1_raw, 2);
    
    delta_matrix = zeros(num_rows, num_cols);
    
    decode_a_format = @(str) parse_hex_c_style(str);
    
    for col = 1:num_cols
        col_name = data1_raw.Properties.VariableNames{col};
        if strcmp(col_name, 'time')
            val1 = str2double(data1_raw{:, col});
            val2 = str2double(data2_raw{:, col});
        else
            str_val1 = data1_raw{:, col};
            str_val2 = data2_raw{:, col};
            
            val1 = zeros(num_rows, 1);
            val2 = zeros(num_rows, 1);
    
            for r = 1:num_rows
                val1(r) = decode_a_format(str_val1{r});
                val2(r) = decode_a_format(str_val2{r});
            end
        end
    
        if strcmp(col_name, 'time')
            delta_matrix(:, col) = val1; 
            continue;
        end
    
        delta_matrix(:, col) = val2 - val1;
    end
    
    headers_delta = data1_raw.Properties.VariableNames;
    fid_out = fopen(file_output, 'wt'); 
    if fid_out == -1
        error('Error : Could not open CSV file');
    end

    fprintf('Writing simulation log in %%a format @ %s...\t', file_output);
    
    join_headers = strjoin(headers_delta, ',');
    fprintf(fid_out, '%s\n', join_headers);
    
    format_line_out = ['%.4f', repmat(',%.5g', 1, length(headers_delta) - 1), '\n'];
    fprintf(fid_out, format_line_out, delta_matrix.');
    fclose(fid_out);

    fprintf('[done]\n');
end


function val = parse_hex_c_style(str)
        str = strtrim(str);
        if isempty(str) || strcmp(str, '0x0p+0') || strcmp(str, '0')
            val = 0;
            return;
        end
        if strcmp(str, 'nan') || strcmp(str, 'NaN'), val = NaN; return; end
        if strcmp(str, 'inf'), val = Inf; return; end
        if strcmp(str, '-inf'), val = -Inf; return; end
        
        try
            tokens = regexp(str, '^([+\x2D]?)(0x[0-9a-fA-F]+)\.?([0-9a-fA-F]*)[pP]([+\x2D]?[0-9]+)$', 'tokens');
            
            if isempty(tokens)
                val = 0;
                return;
            end
            
            parts = tokens{1};
            sign_str = parts{1};
            int_hex = parts{2};    
            frac_hex = parts{3};  
            exponent = str2double(parts{4});
            mantissa = hex2dec(int_hex(3:end)); 
    
            for i = 1:length(frac_hex)
                mantissa = mantissa + hex2dec(frac_hex(i)) * (16^-i);
            end
    
            val = mantissa * (2^exponent);
            if strcmp(sign_str, '-')
                val = -val;
            end
        catch
            val = 0;
        end
    end