from math import *
import numpy as np


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
    print(f"{"CRITERE":^20} | {"VALEUR":^10} | {"VALIDATION":^12}")
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

    print(f"{"CRITERE":^20} | {"VALEUR":^10} | {"VALIDATION":^12}")
    print('-' * 48)
    print(f"{rmse_criteria:20} | {rmse:>7.3f}    | {rmse_validation:^12}")
    print(f"{max_error_criteria:20} | {max_error:>7.3f}    | {max_error_validation:^12}")
