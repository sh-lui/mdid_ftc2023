package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Collections;

public class PIDController {
    // Hyper params
    private double Kp;
    private double Ki;
    private double Kd;
    private double IErrorThres;

    public double currentP;
    public double currentI;
    public double currentD;

    private double errorSum;
    private double previousError;
    public double previousRunTime;
    public int targetPosition = 0;
    private ArrayList<Double> errorHistory = new ArrayList<Double>(); // previous 10 errors

    public PIDController(double _Kp, double _Ki, double _Kd, double _IErrorThres) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        IErrorThres = _IErrorThres;
    }

    public void reset(int _targetPosition, int _currentPosition, double runTime) {
        errorSum = (double) 0;
        targetPosition = _targetPosition;
        previousError = (double) (_currentPosition - _targetPosition);
        previousRunTime = runTime;
    }


    public double getNextVal(double currentPosition, double runTime) {
        double timeElapsed = runTime - previousRunTime;

        // P
        double error = targetPosition - currentPosition;
        // I
        errorSum += (Math.abs(error) < IErrorThres) ? error * timeElapsed : 0;
        // D
        double errorDer = (error - previousError) / timeElapsed;

        // record legacy values.
        previousError = error;
        previousRunTime = runTime;

        // public variables for debugging and fine tuning.
        currentP = Kp * error;
        currentI = Ki * errorSum;
        currentD = Kd * errorDer;

        return currentP + currentI + currentD;
    }

}


