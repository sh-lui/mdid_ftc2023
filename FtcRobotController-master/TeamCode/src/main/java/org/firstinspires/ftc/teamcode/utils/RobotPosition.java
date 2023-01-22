package org.firstinspires.ftc.teamcode.utils;

public class RobotPosition {
    public double X;
    public double Y;
    public double Theta;

    public RobotPosition(double _X, double _Y, double _Theta) {
        X = _X;
        Y = _Y;
        Theta = _Theta;
    }

    public void incrementValues(double deltaX, double deltaY, double deltaTheta) {
        X += deltaX;
        Y += deltaY;
        Theta += deltaTheta;
    }
}