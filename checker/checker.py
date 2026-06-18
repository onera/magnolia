from math import *
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os


###########################################################################
## REQUIREMENTS DEFINITION
###########################################################################

requirements = dict()
requirements['step'] = {'X/Y': {'R1': {'5% name': '5% settling time', '5% value': 6.0,
                                       '2% name': '2% settling time', '2% value': 7.0,
                                       '1% name': '1% settling time', '1% value': 8.0},
                                'R2': {'name': 'Rise Time', 'value': 4.0},
                                'R3': {'name': 'Overshoot', 'value': 2.0},
                                'R4': {'name': 'Static error', 'value': 1.0}},
                        'Z': {'R1': {'5% name': '5% settling time', '5% value': 6.0,
                                     '2% name': '2% settling time', '2% value': 7.0,
                                     '1% name': '1% settling time', '1% value': 8.0},
                              'R2': {'name': 'Rise Time', 'value': 4.0},
                              'R3': {'name': 'Overshoot', 'value': 2.0},
                              'R4': {'name': 'Static error', 'value': 1.0}},
                        'phi/theta': {'R1': {'5% name': '5% settling time', '5% value': 1.0,
                                             '2% name': '2% settling time', '2% value': 1.5,
                                             '1% name': '1% settling time', '1% value': 2.0},
                                      'R2': {'name': 'Rise Time', 'value': 0.8},
                                      'R3': {'name': 'Overshoot', 'value': 2.0},
                                      'R4': {'name': 'Static error', 'value': 1.0}},
                        'psi': {'R1': {'5% name': '5% settling time', '5% value': 3.0,
                                       '2% name': '2% settling time', '2% value': 3.5,
                                       '1% name': '1% settling time', '1% value': 4.0},
                                'R2': {'name': 'Rise Time', 'value': 2.5},
                                'R3': {'name': 'Overshoot', 'value': 2.0},
                                'R4': {'name': 'Static error', 'value': 1.0}}}

requirements['trajectory'] = {'X/Y': {'R1': {'name': 'RMSE', 'value': 0.1},
                                      'R2': {'name': 'Max error', 'value': 0.15}},
                              'Z': {'R1': {'name': 'RMSE', 'value': 0.1},
                                    'R2': {'name': 'Max error', 'value': 0.15}},
                              'psi': {'R1': {'name': 'RMSE', 'value': 0.1},
                                      'R2': {'name': 'Max error', 'value': 0.5}}}

###########################################################################
## STEP
###########################################################################

def get_settling (x, t, th):
    """
    Returns: settling time of x at th%
    
    Parameter x: variable tested
    Parameter t: time list
    Parameter th: settling threshold (ex : 5% of steady state value)
    """
    x_steady_state = x[-1]
    settling_time_5_value = abs(x_steady_state*th/100)
    indexes = [i for i in range(len(x)) if abs(x[i]-x_steady_state) > settling_time_5_value]

    return float(t[indexes[-1]])

def get_rise_time (x, t):
    """
    Returns: rise time of x
    
    Parameter x: variable tested
    Parameter t: time list
    """
    x_steady_state = x[-1]

    rise_time_10_value = 0.1 * x_steady_state
    indexes = [i for i in range(len(x)) if x[i] < rise_time_10_value]
    rise_time_10 = t[indexes[-1]]

    rise_time_90_value = 0.9 * x_steady_state
    indexes = [i for i in range(len(x)) if x[i] < rise_time_90_value]
    rise_time_90 = t[indexes[-1]]
    
    return float(rise_time_90 - rise_time_10)

def get_overshoot (x, t):
    """
    Returns: overshoot of x
    
    Parameter x: variable tested
    Parameter t: time list
    """
    x_steady_state = x[-1]
    x_max = max(x)

    overshoot = (x_max - x_steady_state)/x_steady_state*100
    return float(max(0,overshoot))

def get_static_error (x, ref):
    """
    Returns: static error of x compared to ref
    
    Parameter x: variable tested
    Parameter ref: x reference value
    """
    x_steady_state = x[-1]
    
    return float(abs(ref - x_steady_state)/ref*100)

def step_checker(x, t, ref, threshold, variable):
    if variable == 'X' or variable == 'Y':
        variable = 'X/Y'

    s_time = get_settling (x, t, threshold)
    r_time = get_rise_time (x, t)
    ov = get_overshoot (x, t)
    s_error = get_static_error (x, ref)

    s_time_criteria = requirements['step'][variable]['R1'][f"{int(threshold)}% name"]
    r_time_criteria = requirements['step'][variable]['R2']['name']
    ov_criteria = requirements['step'][variable]['R3']['name']                   
    s_error_criteria = requirements['step'][variable]['R4']['name']

    s_time_validation = str(s_time<requirements['step'][variable]['R1'][f"{int(threshold)}% value"]).upper()
    r_time_validation = str(r_time<requirements['step'][variable]['R2']['value']).upper()
    ov_validation = str(ov<requirements['step'][variable]['R3']['value']).upper()
    s_error_validation = str(s_error<requirements['step'][variable]['R4']['value']).upper()

    print(f"{float(t[-1])}s step response properties for a step of {float(ref)}\n")
    print(f"{"NAME":^20} | {"VALUE":^10} | {"STATUS":^12}")
    print('-' * 48)
    print(f"{s_time_criteria:20} | {float(s_time):>7.3f} s  | {s_time_validation:^12}")
    print(f"{r_time_criteria:20} | {float(r_time):>7.3f} s  | {r_time_validation:^12}")
    print(f"{ov_criteria:20} | {float(ov):>7.3f} %  | {ov_validation:^12}")
    print(f"{s_error_criteria:20} | {float(s_error):>7.3f} %  | {s_error_validation:^12}")

###########################################################################
# TRAJECTORY
###########################################################################

def get_rmse (x , ref):
    x = np.array(x).flatten()
    ref = np.array(ref).flatten()
    n = min(len(x), len(ref))
    return np.sqrt(np.mean((np.array(x[:n]) - np.array(ref[:n]))**2))

def get_max_error (x, ref):
    x = np.array(x).flatten()
    ref = np.array(ref).flatten()
    n = min(len(x), len(ref))
    return np.max(np.abs(np.array(x[:n])-np.array(ref[:n])))

def trajectory_checker(x, ref, variable):
    if variable == 'X' or variable == 'Y':
        variable = 'X/Y'

    rmse = get_rmse(x, ref)
    max_error = get_max_error(x, ref)
    
    rmse_criteria = requirements['trajectory'][variable]['R1']['name']
    max_error_criteria = requirements['trajectory'][variable]['R2']['name']

    rmse_validation = str(rmse<requirements['trajectory'][variable]['R1']['value']).upper()
    max_error_validation = str(max_error<requirements['trajectory'][variable]['R2']['value']).upper()

    print(f"{"NAME":^20} | {"VALUE":^10} | {"STATUS":^12}")
    print('-' * 48)
    print(f"{rmse_criteria:20} | {rmse:>7.3f}    | {rmse_validation:^12}")
    print(f"{max_error_criteria:20} | {max_error:>7.3f}    | {max_error_validation:^12}")

###########################################################################
# PLOTTING
###########################################################################

def plot_sim_results(csv_filepath="c_implementation/debug/sim_log.csv"):
    if not os.path.exists(csv_filepath):
        # Fallback de chemin si lancé depuis un autre dossier
        csv_filepath = "debug/sim_log.csv"
        if not os.path.exists(csv_filepath):
            print(f"Erreur : Le fichier {csv_filepath} est introuvable.")
            return

    df = pd.read_csv(csv_filepath)

    # Configuration du style Scope
    plt.rcParams['grid.color'] = '#404040'
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.linewidth'] = 0.5
    
    # =========================================================================
    # 1. DASHBOARD
    # =========================================================================
    fig_dash, axs = plt.subplots(3, 2, figsize=(16, 9))
    fig_dash.canvas.manager.set_window_title('Dashboard: Control & Estimation Overview')
    
    for ax in axs.flat:
        ax.set_facecolor("#ffffff") 
        ax.grid(True)
        ax.tick_params(colors='black')

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
    axs[0, 0].legend(loc='upper left', ncol=3, fontsize='x-small', facecolor='none', edgecolor='none', labelcolor='white')

    # 1.2 Velocity Tracking 
    axs[0, 1].step(df['time'], df['est_dx'], '-', color='#1f77b4', linewidth=1.2, label='Est dx', where='post')
    axs[0, 1].step(df['time'], df['est_dy'], '-', color='#ff7f0e', linewidth=1.2, label='Est dy', where='post')
    axs[0, 1].step(df['time'], df['est_dz'], '-', color='#2ca02c', linewidth=1.2, label='Est dz', where='post')
    axs[0, 1].plot(df['time'], df['dx'], '-', color='#9467bd', linewidth=1.8, label='Real dx')
    axs[0, 1].plot(df['time'], df['dy'], '-', color='#8c564b', linewidth=1.8, label='Real dy')
    axs[0, 1].plot(df['time'], df['dz'], '-', color='#e377c2', linewidth=1.8, label='Real dz')
    axs[0, 1].set_ylabel('Velocity [m/s]')
    axs[0, 1].set_title('Velocity Tracking (dx, dy, dz)')
    axs[0, 1].legend(loc='upper left', ncol=2, fontsize='x-small', facecolor='none', edgecolor='none', labelcolor='white')

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
    axs[1, 0].legend(loc='upper left', ncol=3, fontsize='x-small', facecolor='none', edgecolor='none', labelcolor='white')

    # 1.4 Angular Rates 
    axs[1, 1].step(df['time'], df['est_p'], '-', color='#1f77b4', linewidth=1.2, label='Est p', where='post') 
    axs[1, 1].step(df['time'], df['est_q'], '-', color='#ff7f0e', linewidth=1.2, label='Est q', where='post')
    axs[1, 1].step(df['time'], df['est_r'], '-', color='#2ca02c', linewidth=1.2, label='Est r', where='post')
    axs[1, 1].plot(df['time'], df['p'], '-', color='#9467bd', linewidth=1.8, label='Real p')
    axs[1, 1].plot(df['time'], df['q'], '-', color='#8c564b', linewidth=1.8, label='Real q')
    axs[1, 1].plot(df['time'], df['r'], '-', color='#e377c2', linewidth=1.8, label='Real r')
    axs[1, 1].set_ylabel('Angular Rate [rad/s]')
    axs[1, 1].set_title('Angular Rates (p, q, r)')
    axs[1, 1].legend(loc='upper left', ncol=2, fontsize='x-small', facecolor='none', edgecolor='none', labelcolor='white')

    # 1.5 Thrust 
    axs[2, 0].step(df['time'], df['cmd_T'], '-', color='#1f77b4', linewidth=1.2, label='Cmd T', where='post')
    axs[2, 0].plot(df['time'], df['real_T'], '-', color='#ff7f0e', linewidth=1.8, label='Real T_m (Motors)')
    axs[2, 0].set_ylabel('Thrust [N]')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_title('Thrust: Commanded vs Real (T vs T_m)')
    axs[2, 0].legend(loc='upper left', facecolor='none', edgecolor='none', labelcolor='white')

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
    axs[2, 1].legend(loc='upper left', ncol=2, fontsize='x-small', facecolor='none', edgecolor='none', labelcolor='white')

    plt.tight_layout()

    # =========================================================================
    # 2. FIGURES PLEIN ÉCRAN
    # =========================================================================
    
    # 2.1 Position 
    plt.figure('Position Tracking', figsize=(12, 6))
    plt.step(df['time'], df['ref_x'], '-', linewidth=1.2, label='Ref x')
    plt.step(df['time'], df['ref_y'], '-', linewidth=1.2, label='Ref y')
    plt.step(df['time'], df['ref_z'], '-', linewidth=1.2, label='Ref z')
    plt.step(df['time'], df['est_x'], '-', linewidth=1.4, label='Est x')
    plt.step(df['time'], df['est_y'], '-', linewidth=1.4, label='Est y')
    plt.step(df['time'], df['est_z'], '-', linewidth=1.4, label='Est z')
    plt.plot(df['time'], df['x'], '-', linewidth=2.0, label='Real x')
    plt.plot(df['time'], df['y'], '-', linewidth=2.0, label='Real y')
    plt.plot(df['time'], df['z'], '-', linewidth=2.0, label='Real z')
    plt.grid(True)
    plt.title('Position Tracking (x, y, z)', fontsize=16, fontweight='bold', pad=15)
    plt.ylabel('Position [m]', fontsize=14, fontweight='bold')
    plt.xlabel('Time [s]', fontsize=14, fontweight='bold')
    plt.legend(loc='upper left', ncol=9, frameon=False)
    
    # 2.2 Velocity 
    plt.figure('Velocity', figsize=(12, 6))
    plt.step(df['time'], df['est_dx'], '-', linewidth=1.4, label='Est dx')
    plt.step(df['time'], df['est_dy'], '-', linewidth=1.4, label='Est dy')
    plt.step(df['time'], df['est_dz'], '-', linewidth=1.4, label='Est dz')
    plt.plot(df['time'], df['dx'], '-', linewidth=2.0, label='Real dx')
    plt.plot(df['time'], df['dy'], '-', linewidth=2.0, label='Real dy')
    plt.plot(df['time'], df['dz'], '-', linewidth=2.0, label='Real dz')
    plt.grid(True)
    plt.ylabel('Velocity [m/s]')
    plt.xlabel('Time [s]')
    plt.title('Velocity (dx, dy, dz)')
    plt.legend(loc='upper left', ncol=2, frameon=False)

    # 2.3 Attitude 
    plt.figure('Attitude Tracking', figsize=(12, 6))
    plt.step(df['time'], df['ref_phi'], '-', linewidth=1.2, label=r'Ref $\phi$')
    plt.step(df['time'], df['ref_theta'], '-', linewidth=1.2, label=r'Ref $\theta$')
    plt.step(df['time'], df['ref_psi'], '-', linewidth=1.2, label=r'Ref $\psi$')
    plt.step(df['time'], df['est_phi'], '-', linewidth=1.4, label=r'Est $\phi$')
    plt.step(df['time'], df['est_theta'], '-', linewidth=1.4, label=r'Est $\theta$')
    plt.step(df['time'], df['est_psi'], '-', linewidth=1.4, label=r'Est $\psi$')
    plt.plot(df['time'], df['phi'], '-', linewidth=2.0, label=r'Real $\phi$')
    plt.plot(df['time'], df['theta'], '-', linewidth=2.0, label=r'Real $\theta$')
    plt.plot(df['time'], df['psi'], '-', linewidth=2.0, label=r'Real $\psi$')
    plt.grid(True)
    plt.ylabel('Attitude [rad]')
    plt.xlabel('Time [s]')
    plt.title(r'Attitude Tracking ($\phi$, $\theta$, $\psi$)')
    plt.legend(loc='upper left', ncol=3, frameon=False)

    # 2.4 Angular Rates 
    plt.figure('Angular Rates', figsize=(12, 6))
    plt.step(df['time'], df['est_p'], '-', linewidth=1.4, label='Est p')
    plt.step(df['time'], df['est_q'], '-', linewidth=1.4, label='Est q')
    plt.step(df['time'], df['est_r'], '-', linewidth=1.4, label='Est r')
    plt.plot(df['time'], df['p'], '-', linewidth=2.0, label='Real p')
    plt.plot(df['time'], df['q'], '-', linewidth=2.0, label='Real q')
    plt.plot(df['time'], df['r'], '-', linewidth=2.0, label='Real r')
    plt.grid(True)
    plt.ylabel('Angular Rate [rad/s]')
    plt.xlabel('Time [s]')
    plt.title('Angular Rates (p, q, r)')
    plt.legend(loc='upper left', ncol=2, frameon=False)

    # 2.5 Thrust 
    plt.figure('Thrust', figsize=(12, 6))
    plt.step(df['time'], df['cmd_T'], '-', linewidth=1.4, label='Cmd T')
    plt.plot(df['time'], df['real_T'], '-', linewidth=2.0, label='Real T_m (Motors)')
    plt.grid(True)
    plt.ylabel('Thrust [N]')
    plt.xlabel('Time [s]')
    plt.title('Thrust: Commanded vs Real (T vs T_m)')
    plt.legend(loc='upper left', frameon=False)

    # 2.6 Control Torques 
    plt.figure('Control Torques', figsize=(12, 6))
    plt.step(df['time'], df['cmd_tau_phi'], '-', linewidth=1.4, label=r'$\tau_{\phi}$ cmd')
    plt.step(df['time'], df['cmd_tau_theta'], '-', linewidth=1.4, label=r'$\tau_{\theta}$ cmd')
    plt.step(df['time'], df['cmd_tau_psi'], '-', linewidth=1.4, label=r'$\tau_{\psi}$ cmd')
    plt.plot(df['time'], df['real_tau_phi'], '-', linewidth=2.0, label=r'$\tau_{\phi}$ real')
    plt.plot(df['time'], df['real_tau_theta'], '-', linewidth=2.0, label=r'$\tau_{\theta}$ real')
    plt.plot(df['time'], df['real_tau_psi'], '-', linewidth=2.0, label=r'$\tau_{\psi}$ real')
    plt.grid(True)
    plt.ylabel('Torque [N.m]')
    plt.xlabel('Time [s]')
    plt.title(r'Control Torques: Commanded vs Real ($\tau$ vs $\tau_m$)')
    plt.legend(loc='upper left', ncol=2, frameon=False)

    plt.show()

if __name__ == '__main__':
    plot_sim_results()
