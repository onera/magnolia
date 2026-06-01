#!/bin/bash
echo "===================================================="
echo "              EXECUTION OF ALL DEBUGS               "
echo "===================================================="

# 1. Motors
echo "[1/7] Compiling Motors..."
gcc -Wall c_implementation/debug/debug_motors.c c_implementation/src/motors.c c_implementation/src/param.c -lm -o c_implementation/build/debug_motors && ./c_implementation/build/debug_motors
if [ $? -ne 0 ]; then exit 1; fi

# 2. Plant
echo "[2/7] Compiling Plant..."
gcc -Wall c_implementation/debug/debug_plant.c c_implementation/src/plant.c c_implementation/src/param.c -lm -o c_implementation/build/debug_plant && ./c_implementation/build/debug_plant
if [ $? -ne 0 ]; then exit 1; fi

# 3. LQI
echo "[3/7] Compiling LQI..."
gcc -Wall c_implementation/debug/debug_lqi.c c_implementation/src/lqi.c c_implementation/src/param.c -lm -o c_implementation/build/debug_lqi && ./c_implementation/build/debug_lqi
if [ $? -ne 0 ]; then exit 1; fi

# 4. Kalman
echo "[4/7] Compiling Kalman..."
gcc -Wall c_implementation/debug/debug_kalman.c c_implementation/src/kalman.c c_implementation/src/param.c c_implementation/src/utils.c -lm -o c_implementation/build/debug_kalman && ./c_implementation/build/debug_kalman
if [ $? -ne 0 ]; then exit 1; fi

# 5. Sensors
echo "[5/7] Compiling Sensors..."
gcc -Wall c_implementation/debug/debug_sensors.c c_implementation/src/sensors.c c_implementation/src/param.c c_implementation/src/utils.c -lm -o c_implementation/build/debug_sensors && ./c_implementation/build/debug_sensors

# 6. Mahony
echo "[6/7] Compiling Mahony Wrapper..."
gcc -Wall c_implementation/debug/debug_mahony.c c_implementation/src/mahony_wrapper.c c_implementation/lib/MahonyAHRS/MahonyAHRS.c c_implementation/src/param.c -lm -o c_implementation/build/debug_mahony && ./c_implementation/build/debug_mahony

# 7. MPC + OSQP Workspace
echo "[7/7] Compiling MPC Wrapper with Real OSQP Workspace..."
gcc -Wall c_implementation/debug/debug_mpc.c c_implementation/src/mpc_wrapper.c c_implementation/lib/osqp_c_code/src/osqp/*.c c_implementation/src/param.c -lm -o c_implementation/build/debug_mpc && ./c_implementation/build/debug_mpc

echo "===================================================="
echo "          ALL DEBUGS PASSED SUCCESSFULLY!           "
echo "===================================================="