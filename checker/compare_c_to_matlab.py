import os
import re
import pandas as pd
import numpy as np

def parse_hex_c_style(val_str):
    if not isinstance(val_str, str):
        return float(val_str)
    
    val_str = val_str.strip()
    if not val_str or val_str == "0x0p+0" or val_str == "0":
        return 0.0
    
    val_lower = val_str.lower()
    if "nan" in val_lower:
        return float('nan')
    if "inf" in val_lower:
        return float('-inf') if val_lower.startswith('-') else float('inf')
    
    try:
        return float.fromhex(val_str)
    except ValueError:
        try:
            if 'p' in val_str and '.' not in val_str.split('p')[0]:
                parts = val_str.split('p')
                val_str = parts[0] + '.0p' + parts[1]
            return float.fromhex(val_str)
        except:
            return 0.0

def compare_c_to_matlab():
    file1 = 'c_Castle_hex.csv'
    file2 = 'matlab_Castle_hex.csv'
    file_output = 'log_delta.csv'
    
    print("Loading and parsing CSV files...")
    data1_raw = pd.read_csv(file1, dtype=str)
    data2_raw = pd.read_csv(file2, dtype=str)
    
    if data1_raw.shape != data2_raw.shape:
        raise ValueError(f"Error: The two CSV files do not have the same dimensions ({data1_raw.shape} vs {data2_raw.shape}).")
 
    v_parse_hex = np.vectorize(parse_hex_c_style, otypes=[float])
    
    delta_df = pd.DataFrame(index=data1_raw.index, columns=data1_raw.columns)
    
    for col in data1_raw.columns:
        if col == 'time':

            val1 = pd.to_numeric(data1_raw[col], errors='coerce').values
            val2 = pd.to_numeric(data2_raw[col], errors='coerce').values
            delta_df[col] = val1 
        else:
 
            val1 = v_parse_hex(data1_raw[col].values)
            val2 = v_parse_hex(data2_raw[col].values)
            delta_df[col] = val2 - val1
            
    print(f"Writing simulation log @ {file_output}...")

    with open(file_output, 'w', newline='') as f:

        f.write(",".join(delta_df.columns) + "\n")

        time_col_idx = delta_df.columns.get_loc('time')

        formats = [f"{{:.5g}}" for _ in range(len(delta_df.columns))]
        formats[time_col_idx] = "{:.4f}"
        line_format = ",".join(formats) + "\n"

        for row in delta_df.itertuples(index=False):
            f.write(line_format.format(*row))
            
    print("[done]")

if __name__ == "__main__":
    compare_c_to_matlab()