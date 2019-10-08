# MPU6050_Calibration
Files to calibrate an MPU6050 using Arduino and Python using method outlined in:

J. Rohac, M. Sipos and J. Simanek, "Calibration of low-cost triaxial inertial sensors", IEEE Instrumentation \& Measurement Magazine, vol. 18, no. 6, pp. 32-38, 2015. doi: 10.1109/mim.2015.7335836.

Uses MPU6050 and I2C libraries by Jeff Rowberg at: https://github.com/jrowberg/i2cdevlib

Transformation from uncalibrated to calibrated reading: **a_calibrated** = **NS(a_uncal - b)** see the report for details 

Steps to get SEM parameters:
    1. Place the MPU6050 securely in the calibration cube  (design under ./3D)
    2. Connect the MPU6050 to an Arduino UNO using I2C and wait 5 - 15 min for it to warm up
    3. Load the calibration script in Arduino/src/MPU6050_calibrate and open up a serial monitor
    4. Place the cube in a static position  
    5. Repeat the previous step for 24 (or more) different rotations 
    6. Take down the averaged acceleration vectors and save in a text file called 'calibration.txt' under /Data
    7. Run the Python/accelCalibration.py script
    8. Record the SEM parameters and use it in a transformation on every acceleration reading
    
