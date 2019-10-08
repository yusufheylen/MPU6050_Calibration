#!/usr/bin/env python3
# -*- codind: utf-8 -*-
"""
Created on Mon Oct  7 11:17:36 2019
@brief: Script to calculate the SEM parameters for the MPU6050 calibration 
@author: yusufheylen
"""

import numpy as np
from scipy.optimize import minimize as opt

LATITUDE = -33.95 #Cape Town, South Africa
gval = 9.806 - 0.5*(9.832 -  9.780)*np.cos(2*(LATITUDE)*np.pi/(180))
G = 1

def rmse(accelMatrix):
    """
        Calculate the Root Mean Square Error.
        @arg: accelMatrix, (nx3) matrix of Averaged Ax, Ay, Az for a different orientaion
        @arg: n, matrix dimension / number of rotations used
        @return: the root mean squared error
    """
    s = 0
    n = 0
    for accelVector in accelMatrix:
        n +=1
        s = s + ( (accelVector[0]**2 + accelVector[1]**2 + accelVector[2]**2)**0.5 - G)**2
    return (s/n)**0.5
    
def correction(accelMatrix, N, S, b):
    """
        N - 2D nx3 array of orthogonality correction factors
        S -  2D nx3 array of scaling factors
        accleMatrix = 2D nx3 array acceleration readings (each row is one)
        b - offset row vector 
    """
    N = np.array(N)
    S = np.array(S)
    b = np.array(b)
    accelCorrected = []
    for accelVector in accelMatrix:
        vector = np.array(accelVector, dtype='float64')
        accelCorrected.append( (np.hstack(N@S@np.vstack(np.subtract(vector, b) ) )).tolist())
    return accelCorrected
        

def minimize(x, accelMatrix):
    N = [
        [1, 0, 0],
        [x[0], 1, 0],
        [x[1], x[2], 1]
        ]
    
    S = [
        [x[3], 0, 0],
        [0, x[4], 0],
        [0, 0, x[5]]
        ]
    
    b = [x[6],x[7],x[8]]
    
    estimate = correction(accelMatrix, N, S, b)
    return rmse(estimate)

def main():
    with open('../Data/calibration.txt') as f:  #calibration readings file
        x0 = [0, 0, 0, 1, 1, 1, 0, 0, 0]       #% Structure of vector: x0 = [ALFAxy, ALFAzx, ALFAzy, Sx, Sy, Sz, bx, by, bz]
        accelData = []
        for row in f:
            vector = row.split(',')
            vector[0] = float(vector[0])
            vector[1] = float(vector[1])
            vector[2] = float(vector[2])
            accelData.append(vector)
        x = opt((lambda x: minimize(x, accelData)), x0, method='BFGS').x

        N = [
        [1, 0, 0],
        [x[0], 1, 0],
        [x[1], x[2], 1]
        ]
        
        S = [
        [x[3], 0, 0],
        [0, x[4], 0],
        [0, 0, x[5]]
        ]
        
        b = [x[6],x[7],x[8]]
        
        print("\nN =\n" + str(np.array(N)))
        print("\nS =\n" + str(np.array(S)))
        print("\nb = " + str(np.array(b)))

        print("\nRMSE before calibration: " + '{:5f}'.format(rmse(accelData)*gval) +" m/s^2")
        print("RMSE after calibration: " + '{:5f}'.format(  rmse(correction(accelData, N, S, b) )*gval ) + " m/s^2")
        



if __name__ == "__main__":
    main()