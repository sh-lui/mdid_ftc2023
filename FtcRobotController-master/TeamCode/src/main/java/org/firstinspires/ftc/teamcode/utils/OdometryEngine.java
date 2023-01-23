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

    private int encoder1PreviousPosition = 0;
    private int encoder2PreviousPosition = 0;
    private int encoder3PreviousPosition = 0;

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

    public void resetEncoderPosition(int pos1, int pos2, int pos3) {
        encoder1PreviousPosition = pos1;
        encoder2PreviousPosition = pos2;
        encoder3PreviousPosition = pos3;
    }

    public void resetPosition(RobotPosition _currentPosition) {
        currentPosition = _currentPosition;
    }

    public RobotPosition getCurrentPosition() {
        return currentPosition;
    }

    public void updatePosition(int encoder1Position, int encoder2Position, int encoder3Position) throws Exception {
        _assertConfigured();

        int encoder1Ticks,  encoder2Ticks, encoder3Ticks;
        encoder1Ticks = encoder1PreviousPosition - encoder1Position;
        encoder2Ticks = encoder2PreviousPosition - encoder2Position;
        encoder3Ticks = encoder3PreviousPosition - encoder3Position;

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
