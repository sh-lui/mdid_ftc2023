package org.firstinspires.ftc.teamcode.utils;

/*
This engine utilizes the three-dead-wheel setup.
 */


public class OdometryEngine {
    private RobotPosition currentPosition;

    private boolean configured = false;
    private double encoderRadius;
    private double encoderTicksPerRevolution;
    private double encoderVerticalSpan;
    private double encoderHorizontalSpan;

    public OdometryEngine(RobotPosition _currentPosition) {
        currentPosition = _currentPosition;
    }

    public void configureEncoder(double _encoderRadius, double _encoderTicksPerRevolution, double _encoderVerticalSpan, double _encoderHorizontalSpan) {
        encoderRadius = _encoderRadius;
        encoderTicksPerRevolution = _encoderTicksPerRevolution;
        encoderVerticalSpan = _encoderVerticalSpan;
        encoderHorizontalSpan = _encoderHorizontalSpan;
        configured = true;
    }

    public void resetPosition(RobotPosition _currentPosition) {
        currentPosition = _currentPosition;
    }

    public RobotPosition getCurrentPosition() throws Exception {
        _assertConfigured();
        return currentPosition;
    }

    public void updatePosition(int encoder1Ticks, int encoder2Ticks, int encoder3Ticks) throws Exception {
        _assertConfigured();

        double deltaX, deltaY, deltaTheta;

        double deltaE1 = _getEncoderDistanceFromTicks(encoder1Ticks);
        double deltaE2 = _getEncoderDistanceFromTicks(encoder2Ticks);
        double deltaE3 = _getEncoderDistanceFromTicks(encoder3Ticks);

        deltaX = (deltaE1 + deltaE2) / 2;
        deltaY = deltaE3  - encoderHorizontalSpan * (deltaE2 - deltaE1) / encoderVerticalSpan;
        deltaTheta = (deltaE2 - deltaE1) / encoderVerticalSpan;

        currentPosition.incrementValues(deltaX, deltaY, deltaTheta);
    }

    private void _assertConfigured() throws Exception {
        if (!configured) {
            throw new Exception("Encoder not configured! Please configure the encoders using the configureEncoder method.");
        }
    }

    private double _getEncoderDistanceFromTicks(int ticks) {
        return 2 * Math.PI * encoderRadius * (ticks / encoderTicksPerRevolution);
    }

}
