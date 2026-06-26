import csv
import math
import os
import matplotlib.pyplot as plt

requirements = {
    'step': {
        'X/Y': {'settling_5%': 6.0, 'rise_time': 4.0, 'overshoot': 2.0, 'static_error': 1.0},
        'Z':   {'settling_5%': 6.0, 'rise_time': 4.0, 'overshoot': 2.0, 'static_error': 1.0},
        'psi': {'settling_5%': 3.0, 'rise_time': 2.5, 'overshoot': 2.0, 'static_error': 1.0}
    },
    'trajectory': {
        'X/Y': {'rmse': 0.1, 'max_error': 0.15},
        'Z':   {'rmse': 0.1, 'max_error': 0.15},
        'psi': {'rmse': 0.1, 'max_error': 0.5}
    }
}

def get_settling(x, t, th=5.0):
    x_steady_state = x[-1]
    settling_threshold_val = abs(x_steady_state * th / 100)
    
    last_out_index = -1
    for i in range(len(x)):
        if abs(x[i] - x_steady_state) > settling_threshold_val:
            last_out_index = i
            
    return float(t[last_out_index]) if last_out_index != -1 else 0.0

def get_rise_time(x, t):
    x_steady_state = x[-1]
    
    rise_10_val = 0.1 * x_steady_state
    rise_time_10 = t[0]
    for i in range(len(x)):
        if x[i] < rise_10_val:
            rise_time_10 = t[i]

    rise_90_val = 0.9 * x_steady_state
    rise_time_90 = t[-1]
    for i in range(len(x)):
        if x[i] < rise_90_val:
            rise_time_90 = t[i]
            
    return float(rise_time_90 - rise_time_10)

def get_overshoot(x):
    x_steady_state = x[-1]
    if x_steady_state == 0: return 0.0
    x_max = max(x) if x_steady_state >= 0 else min(x)
    overshoot = (x_max - x_steady_state) / x_steady_state * 100
    return float(max(0, overshoot))

def get_static_error(x, ref):
    x_steady_state = x[-1]
    if ref == 0: return float(abs(x_steady_state))
    return float(abs(ref - x_steady_state) / ref * 100)

def get_req_category(variable):
    var_l = variable.lower()
    if var_l in ['x', 'y']: return 'X/Y'
    if var_l == 'z': return 'Z'
    if var_l == 'psi': return 'psi'
    return None

def load_csv_to_dict(csv_filepath):
    data = {}
    with open(csv_filepath, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        for h in headers:
            data[h] = []
        for row in reader:
            for h in headers:
                val_str = row[h].strip()
                if val_str.startswith('0x') or val_str.startswith('-0x'):
                    data[h].append(float.fromhex(val_str))
                else:
                    data[h].append(float(val_str))
    return data


def check_step(data, variable, filename, source):
    for var in variable:
        req_cat = get_req_category(var)
        x = data[var.lower()]
        t = data['time']
        ref_val = data[f'ref_{var.lower()}'][-1]
        
        s_time = get_settling(x, t, 5.0)
        r_time = get_rise_time(x, t)
        ov = get_overshoot(x)
        s_error = get_static_error(x, ref_val)
        
        limits = requirements['step'][req_cat]
        
        s_time_valid  = "PASS" if s_time < limits['settling_5%'] else "FAIL"
        r_time_valid  = "PASS" if r_time < limits['rise_time'] else "FAIL"
        ov_valid      = "PASS" if ov < limits['overshoot'] else "FAIL"
        s_error_valid = "PASS" if s_error < limits['static_error'] else "FAIL"
        
        print(f"File: {filename} [{source}] | Type: Step {var.upper()} ({t[-1]:.1f}s)")
        print(f"{"REQUIREMENT":<25} | {"VALUE":^12} | {"LIMIT":^12} | {"STATUS":^10}")
        print('-' * 68)
        print(f"{"5% Settling Time":<25} | {s_time:>10.2f} s | {limits['settling_5%']:>10.2f} s | {s_time_valid:^10}")
        print(f"{"Rise Time":<25} | {r_time:>10.2f} s | {limits['rise_time']:>10.2f} s | {r_time_valid:^10}")
        print(f"{"Overshoot":<25} | {ov:>10.2f} % | {limits['overshoot']:>10.2f} % | {ov_valid:^10}")
        print(f"{"Static Error":<25} | {s_error:>10.2f} % | {limits['static_error']:>10.2f} % | {s_error_valid:^10}")
        print("=" * 68 + "\n")

def check_trajectory(data, filename, source):
    t = data['time']
    print(f"File: {filename} [{source}] | Type: Trajectory Tracking ({t[-1]:.1f}s)")
    print(f"{"REQUIREMENT":<25} | {"VALUE":^12} | {"LIMIT":^12} | {"STATUS":^10}")
    print('-' * 68)
    
    for var in ['x', 'y', 'z']:
        req_cat = get_req_category(var)
        x_data = data[var]
        ref_data = data[f'ref_{var}']
        
        errors = [x_d - r_d for x_d, r_d in zip(x_data, ref_data)]
        rmse = math.sqrt(sum([e**2 for e in errors]) / len(errors)) if errors else 0.0
        max_err = max([abs(e) for e in errors]) if errors else 0.0
        
        limits = requirements['trajectory'][req_cat]
        
        rmse_valid = "PASS" if rmse < limits['rmse'] else "FAIL"
        max_valid  = "PASS" if max_err < limits['max_error'] else "FAIL"
        
        print(f"{var.upper() + ' - RMSE':<25} | {rmse:>12.3f} | {limits['rmse']:>12.3f} | {rmse_valid:^10}")
        print(f"{var.upper() + ' - Max Error':<25} | {max_err:>12.3f} | {limits['max_error']:>12.3f} | {max_valid:^10}")
    print("=" * 68 + "\n")

def check_requirements(csv_filepath):
    csv_filepath = str(csv_filepath)

    if not os.path.exists(csv_filepath):
        print(f"[ERROR] Verification aborted: File '{csv_filepath}' not found.")
        return
    
    filename = os.path.basename(csv_filepath)
    data = load_csv_to_dict(csv_filepath)
    source = 'MATLAB' if 'matlab' in filename.lower() else 'C_CODE'
    filename_lower = filename.lower()
    
    print("\n" + "=" * 68)
    print(f"CHECKING REQUIREMENTS")
    print("=" * 68)
    
    if 'step' in filename_lower:
        var_to_check = 'xyz' if 'stepxyz' in filename_lower else ('x' if 'stepx' in filename_lower else ('y' if 'stepy' in filename_lower else 'z'))
        check_step(data, var_to_check, filename, source)
    else:
        check_trajectory(data, filename, source)

def plot_simulation_results(csv_filepath):
    if not os.path.exists(csv_filepath):
        print(f"[ERROR] Plotting aborted: File '{csv_filepath}' not found.")
        return

    df = load_csv_to_dict(csv_filepath)

    plt.rcParams['grid.color'] = '#404040'
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.linewidth'] = 0.5
    
    fig_dash, axs = plt.subplots(3, 2, figsize=(16, 9))
    title_prefix = os.path.basename(csv_filepath)
    fig_dash.canvas.manager.set_window_title(f'Dashboard: {title_prefix}')
    
    for ax in axs.flat:
        ax.set_facecolor("#ffffff") 
        ax.grid(True)

    # 1.1 Position Tracking
    axs[0, 0].step(df['time'], df['ref_x'], '-', color='#1f77b4', linewidth=1.0, label='Ref x', where='post')
    axs[0, 0].step(df['time'], df['ref_y'], '-', color='#ff7f0e', linewidth=1.0, label='Ref y', where='post')
    axs[0, 0].step(df['time'], df['ref_z'], '-', color='#2ca02c', linewidth=1.0, label='Ref z', where='post')
    axs[0, 0].step(df['time'], df['est_x'], '-', color='#9467bd', linewidth=1.2, label='Est x', where='post')
    axs[0, 0].step(df['time'], df['est_y'], '-', color='#8c564b', linewidth=1.2, label='Est y', where='post')
    axs[0, 0].step(df['time'], df['est_z'], '-', color='#e377c2', linewidth=1.2, label='Est z', where='post')
    axs[0, 0].plot(df['time'], df['x'], '-', color='#bcbd22', linewidth=1.8, label='Real x')
    axs[0, 0].plot(df['time'], df['y'], '-', color='#17becf', linewidth=1.8, label='Real y')
    axs[0, 0].plot(df['time'], df['z'], '-', color='#d62728', linewidth=1.8, label='Real z')
    axs[0, 0].set_ylabel('Position [m]')
    axs[0, 0].set_title('Position Tracking (x, y, z)')
    axs[0, 0].legend(loc='upper left', ncol=3, fontsize='x-small', facecolor='none', edgecolor='none')

    # 1.2 Velocity Tracking
    axs[0, 1].step(df['time'], df['ref_dx'], '--', color='#1f77b4', linewidth=1.0, label='Ref dx', where='post')
    axs[0, 1].step(df['time'], df['ref_dy'], '--', color='#ff7f0e', linewidth=1.0, label='Ref dy', where='post')
    axs[0, 1].step(df['time'], df['ref_dz'], '--', color='#2ca02c', linewidth=1.0, label='Ref dz', where='post')
    axs[0, 1].step(df['time'], df['est_dx'], '-', color='#9467bd', linewidth=1.2, label='Est dx', where='post')
    axs[0, 1].step(df['time'], df['est_dy'], '-', color='#8c564b', linewidth=1.2, label='Est dy', where='post')
    axs[0, 1].step(df['time'], df['est_dz'], '-', color='#e377c2', linewidth=1.2, label='Est dz', where='post')
    axs[0, 1].plot(df['time'], df['dx'], '-', color='#bcbd22', linewidth=1.8, label='Real dx')
    axs[0, 1].plot(df['time'], df['dy'], '-', color='#17becf', linewidth=1.8, label='Real dy')
    axs[0, 1].plot(df['time'], df['dz'], '-', color='#d62728', linewidth=1.8, label='Real dz')
    axs[0, 1].set_ylabel('Velocity [m/s]')
    axs[0, 1].set_title('Velocity Tracking (dx, dy, dz)')
    axs[0, 1].legend(loc='upper left', ncol=3, fontsize='x-small', facecolor='none', edgecolor='none')

    # 1.3 Attitude Tracking
    axs[1, 0].step(df['time'], df['ref_phi'], '-', color='#1f77b4', linewidth=1.0, label=r'Ref $\phi$', where='post')
    axs[1, 0].step(df['time'], df['ref_theta'], '-', color='#ff7f0e', linewidth=1.0, label=r'Ref $\theta$', where='post')
    axs[1, 0].step(df['time'], df['ref_psi'], '-', color='#2ca02c', linewidth=1.0, label=r'Ref $\psi$', where='post')
    axs[1, 0].step(df['time'], df['est_phi'], '-', color='#9467bd', linewidth=1.2, label=r'Est $\phi$', where='post')
    axs[1, 0].step(df['time'], df['est_theta'], '-', color='#8c564b', linewidth=1.2, label=r'Est $\theta$', where='post')
    axs[1, 0].step(df['time'], df['est_psi'], '-', color='#e377c2', linewidth=1.2, label=r'Est $\psi$', where='post')
    axs[1, 0].plot(df['time'], df['phi'], '-', color='#bcbd22', linewidth=1.8, label=r'Real $\phi$')
    axs[1, 0].plot(df['time'], df['theta'], '-', color='#17becf', linewidth=1.8, label=r'Real $\theta$')
    axs[1, 0].plot(df['time'], df['psi'], '-', color='#d62728', linewidth=1.8, label=r'Real $\psi$')
    axs[1, 0].set_ylabel('Attitude [rad]')
    axs[1, 0].set_title(r'Attitude Tracking ($\phi$, $\theta$, $\psi$)')
    axs[1, 0].legend(loc='upper left', ncol=3, fontsize='x-small', facecolor='none', edgecolor='none')

    # 1.4 Angular Rates
    axs[1, 1].step(df['time'], df['est_p'], '-', color='#1f77b4', linewidth=1.2, label='Est p', where='post') 
    axs[1, 1].step(df['time'], df['est_q'], '-', color='#ff7f0e', linewidth=1.2, label='Est q', where='post')
    axs[1, 1].step(df['time'], df['est_r'], '-', color='#2ca02c', linewidth=1.2, label='Est r', where='post')
    axs[1, 1].plot(df['time'], df['p'], '-', color='#9467bd', linewidth=1.8, label='Real p')
    axs[1, 1].plot(df['time'], df['q'], '-', color='#8c564b', linewidth=1.8, label='Real q')
    axs[1, 1].plot(df['time'], df['r'], '-', color='#e377c2', linewidth=1.8, label='Real r')
    axs[1, 1].set_ylabel('Angular Rate [rad/s]')
    axs[1, 1].set_title('Angular Rates (p, q, r)')
    axs[1, 1].legend(loc='upper left', ncol=2, fontsize='x-small', facecolor='none', edgecolor='none')

    # 1.5 Thrust
    axs[2, 0].step(df['time'], df['cmd_T'], '-', color='#1f77b4', linewidth=1.2, label='Cmd T', where='post')
    axs[2, 0].plot(df['time'], df['real_T'], '-', color='#ff7f0e', linewidth=1.8, label='Real T_m (Motors)')
    axs[2, 0].set_ylabel('Thrust [N]')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_title('Thrust: Commanded vs Real (T vs T_m)')
    axs[2, 0].legend(loc='upper left', facecolor='none', edgecolor='none')

    # 1.6 Control Torques
    axs[2, 1].step(df['time'], df['cmd_tau_phi'], '-', color='#1f77b4', linewidth=1.2, label=r'$\tau_{\phi}$ cmd', where='post')
    axs[2, 1].step(df['time'], df['cmd_tau_theta'], '-', color='#ff7f0e', linewidth=1.2, label=r'$\tau_{\theta}$ cmd', where='post')
    axs[2, 1].step(df['time'], df['cmd_tau_psi'], '-', color='#2ca02c', linewidth=1.2, label=r'$\tau_{\psi}$ cmd', where='post')
    axs[2, 1].plot(df['time'], df['real_tau_phi'], '-', color='#9467bd', linewidth=1.8, label=r'$\tau_{\phi}$ real')
    axs[2, 1].plot(df['time'], df['real_tau_theta'], '-', color='#8c564b', linewidth=1.8, label=r'$\tau_{\theta}$ real')
    axs[2, 1].plot(df['time'], df['real_tau_psi'], '-', color='#e377c2', linewidth=1.8, label=r'$\tau_{\psi}$ real')
    axs[2, 1].set_ylabel('Torque [N.m]')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_title(r'Control Torques: Commanded vs Real ($\tau$ vs $\tau_m$)')
    axs[2, 1].legend(loc='upper left', ncol=2, fontsize='x-small', facecolor='none', edgecolor='none')

    plt.tight_layout()
    plt.show()