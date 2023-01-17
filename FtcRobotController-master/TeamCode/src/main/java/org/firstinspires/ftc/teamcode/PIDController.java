package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Collections;

class PIDController {
    // Hyper params
    private double Kp;
    private double Ki;
    private double Kd;

    private double errorSum;
    private double previousError;
    public double previousRunTime;
    private int targetPosition = 0;
    private ArrayList<Double> errorHistory = new ArrayList<Double>(); // previous 10 errors

    // public boolean isActive = false;
    private double errorToleration = 0.05;
    private int errorHistorySize = 10;


    private double sigmoid(double x) {
        return (1 / (1 + Math.pow(Math.E, (-1 * x))));
    }

    public PIDController(double _Kp, double _Ki, double _Kd) {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;

    }

    public void reset(int _targetPosition, int _currentPosition, double runTime) {
        errorSum = (double) 0;
        targetPosition = _targetPosition;
        previousError = (double) (_currentPosition - _targetPosition);
        previousRunTime = runTime;
    }

    public double getNextVal(double currentPosition, double runTime) {
        double timeElapsed = previousRunTime - runTime;

        // P
        double error = targetPosition - currentPosition;
        // I
        errorSum += error * timeElapsed;
        // D
        double errorDer = (error - previousError) / timeElapsed;

        previousError = error;
        previousRunTime = runTime;

        double result = Kp * error + Ki * errorSum + Kd * errorDer;

        // Add to history and determining the stopping condition.

        /* 
        errorHistory.add(error); 
        if (errorHistory.size() > errorHistorySize) { 
            errorHistory.remove(0); 
            if (Collections.max(errorHistory) < errorToleration) { 
                isActive = false; 
            } 
        } 
         */


        return result;
    }

    public double getNextRescaledVal(double currentPosition, double timeElapsed) {
        return sigmoid(getNextVal(currentPosition, timeElapsed));
    }


}


