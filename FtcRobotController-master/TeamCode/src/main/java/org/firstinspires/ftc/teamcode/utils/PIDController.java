package org.firstinspires.ftc.teamcode.utils;

import org.checkerframework.checker.units.qual.K;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Collections;

public class PIDController {
    // Hyper params
    public double Kp;
    public double Ki;
    public double Kd;
    private double IErrorThres;

    public double currentP;
    public double currentI;
    public double currentD;

    private double errorSum;
    private double previousError;
    public double previousRuntime;
    public double targetPosition = 0;
    private ArrayList<Double> errorHistory = new ArrayList<Double>(); // previous 10 errors

    public PIDController(double _Kp, double _Ki, double _Kd, double _IErrorThres) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        IErrorThres = _IErrorThres;
    }

    public void reset(double _targetPosition, double _currentPosition, double runtime) {
        errorSum = (double) 0;
        targetPosition = _targetPosition;
        previousError = (double) (_currentPosition - _targetPosition);
        previousRuntime = runtime;
    }

    public void overridePID(double P, double I, double D) {
        Kp = P;
        Ki = I;
        Kd = D;
    }



    public double getNextVal(double currentPosition, double runtime) {
        double timeElapsed = runtime - previousRuntime;

        // P
        double error = targetPosition - currentPosition;
        // I
        errorSum += (Math.abs(error) < IErrorThres) ? error * timeElapsed : 0;
        // D
        double errorDer = (error - previousError) / timeElapsed;

        // record legacy values.
        previousError = error;
        previousRuntime = runtime;

        // public variables for debugging and fine tuning.
        currentP = Kp * error;
        currentI = Ki * errorSum;
        currentD = Kd * errorDer;

        return currentP + currentI + currentD;
    }

}


