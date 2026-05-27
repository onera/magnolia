#!/bin/bash
echo "===================================================="
echo "              EXECUTION OF ALL TESTS                "
echo "===================================================="

# 1. Motors
echo "[1/5] Compiling Motors..."
gcc -Wall c_implementation/tests/test_motors.c c_implementation/src/motors.c c_implementation/src/param.c -lm -o c_implementation/build/test_motors && ./c_implementation/build/test_motors
if [ $? -ne 0 ]; then exit 1; fi

# 2. Plant
echo "[2/5] Compiling Plant..."
gcc -Wall c_implementation/tests/test_plant.c c_implementation/src/plant.c c_implementation/src/param.c -lm -o c_implementation/build/test_plant && ./c_implementation/build/test_plant
if [ $? -ne 0 ]; then exit 1; fi

# 3. LQI
echo "[3/5] Compiling LQI..."
gcc -Wall c_implementation/tests/test_lqi.c c_implementation/src/lqi.c c_implementation/src/param.c -lm -o c_implementation/build/test_lqi && ./c_implementation/build/test_lqi
if [ $? -ne 0 ]; then exit 1; fi

# 4. Kalman
echo "[4/5] Compiling Kalman..."
gcc -Wall c_implementation/tests/test_kalman.c c_implementation/src/kalman.c c_implementation/src/param.c c_implementation/src/utils.c -lm -o c_implementation/build/test_kalman && ./c_implementation/build/test_kalman
if [ $? -ne 0 ]; then exit 1; fi

# 5. Sensors
echo "[5/5] Compiling Sensors..."
gcc -Wall c_implementation/tests/test_sensors.c c_implementation/src/sensors.c c_implementation/src/param.c c_implementation/src/utils.c -lm -o c_implementation/build/test_sensors && ./c_implementation/build/test_sensors

# 6. Mahony
echo "[6/6] Compiling Mahony Wrapper..."
gcc -Wall c_implementation/tests/test_mahony.c c_implementation/src/mahony_wrapper.c c_implementation/lib/MahonyAHRS/MahonyAHRS.c c_implementation/src/param.c -lm -o c_implementation/build/test_mahony && ./c_implementation/build/test_mahony

echo "===================================================="
echo "          ALL TESTS PASSED SUCCESSFULLY!            "
echo "===================================================="