package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.RobotPosition;
import org.firstinspires.ftc.teamcode.utils.OdometryEngine;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotWheelPower;

public class DriveBase extends BaseComponent {

    public double robotStartX = 0;
    public double robotStartY = 0;
    public double robotStartTheta = 0;

    public double encoderRadius = 19;
    public double encoderTicksPerRevolution = 8192;
    public double encoderVerticalSpan = 340;
    public double encoderHorizontalSpan = 48.53;

    public double angularOffsetTolerance = 0.00001;
    public double translationalOffsetTolerance = 0.000001;

    private double translationalPID_Kp = 0.04;
    private double translationalPID_Ki = 0;
    private double translationalPID_Kd = 0.005;
    private double translationalPID_IErrorThres = 200;

    private double angularPID_Kp = 2;
    private double angularPID_Ki = 0;
    private double angularPID_Kd = 0.05;
    private double angularPID_IErrorThres = 200;


    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor rightFrontMotor = null;

    public DcMotor encoder1 = null;
    public DcMotor encoder2 = null;
    public DcMotor encoder3 = null;

    public OdometryEngine odometryEngine = null;

    public RobotPosition targetPosition = null;
    public PIDController angularPIDController = null;
    public PIDController translationalPIDController = null;

    public double autoTurn;
    public double autoTheta;
    public double autoPower;

    public double angularOffset;

    public DriveBase(DcMotor _leftFrontMotor, DcMotor _leftRearMotor, DcMotor _rightFrontMotor,  DcMotor _rightRearMotor, DcMotor _encoder1, DcMotor _encoder2, DcMotor _encoder3) {
        leftFrontMotor = _leftFrontMotor;
        leftRearMotor = _leftRearMotor;
        rightFrontMotor = _rightFrontMotor;
        rightRearMotor = _rightRearMotor;

        encoder1 = _encoder1;
        encoder2 = _encoder2;
        encoder3 = _encoder3;

        RobotPosition currentPosition = new RobotPosition(robotStartX, robotStartY, robotStartTheta);
        odometryEngine = new OdometryEngine(currentPosition);
        odometryEngine.configureEncoder(encoderRadius, encoderTicksPerRevolution, encoderVerticalSpan, encoderHorizontalSpan);
        odometryEngine.resetEncoderPosition(encoder1.getCurrentPosition(), encoder2.getCurrentPosition(), encoder3.getCurrentPosition());

        translationalPIDController = new PIDController(translationalPID_Kp, translationalPID_Ki, translationalPID_Kd, translationalPID_IErrorThres);
        angularPIDController = new PIDController(angularPID_Kp, angularPID_Ki, angularPID_Kd, angularPID_IErrorThres);
    }

    private RobotWheelPower getMotorPowerFromAngularPower(double theta, double power, double turn) {
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;

        // get the rotated components
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFrontPower = power * cos/max + turn;
        rightFrontPower = power * sin/max - turn;
        leftRearPower = power * sin/max + turn;
        rightRearPower = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1) { // rescale stuff
            leftFrontPower /= power + turn;
            rightFrontPower /= power + turn;
            leftRearPower /= power + turn;
            rightRearPower /= power + turn;
        }

        return new RobotWheelPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }


    public void setTarget(RobotPosition _targetPosition, double runtime) {
        targetPosition = _targetPosition;
        translationalPIDController.reset(0, -1 * _getCartesianDistance(odometryEngine.getCurrentPosition(), targetPosition), runtime);
        angularPIDController.reset(0, -1 * _getAngularOffset(odometryEngine.getCurrentPosition(), targetPosition), runtime);
    }


    public void run(double runtime) {
        try { odometryEngine.updatePosition(encoder1.getCurrentPosition(), encoder2.getCurrentPosition(), encoder3.getCurrentPosition()); } catch (Exception ignore) { }

        if (targetPosition != null) {
            //double theta, power, turn;
            autoTheta = _getTargetAngle(odometryEngine.getCurrentPosition(), targetPosition) + Math.PI/2;
            autoPower = _scaledShiftedSigmoid(translationalPIDController.getNextVal(-1 * _getCartesianDistance(odometryEngine.getCurrentPosition(), targetPosition), runtime), 0.9);

            angularOffset = _getAngularOffset(odometryEngine.getCurrentPosition(), targetPosition);
            autoTurn = _scaledShiftedSigmoid(angularPIDController.getNextVal(-1 * angularOffset, runtime), 0.9);

            _setAngularPower(autoTheta, autoPower, -autoTurn);

            if (targetReached()) {
                dropTarget();
            }
        }
    }

    public boolean targetReached() {
        return _getAngularOffset(odometryEngine.getCurrentPosition(), targetPosition) < angularOffsetTolerance && _getCartesianDistance(odometryEngine.getCurrentPosition(), targetPosition) < translationalOffsetTolerance;
    }

    public void dropTarget() {
        targetPosition = null;
    }

    public void manualSetPower(RobotWheelPower wheelPower) {
        dropTarget();
        _setPower(wheelPower);
    }

    public void manualSetAngularPower(double theta, double power, double turn) {
        dropTarget();
        _setAngularPower(theta, power, turn);
    }

    private double _setHardLimit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private double _normalizeAngle(double angle) {
        return angle % (2 * Math.PI);
    }
    private void _setAngularPower(double theta, double power, double turn) {
        RobotWheelPower wheelPower = getMotorPowerFromAngularPower(theta, power, turn);
        _setPower(wheelPower);

    }

    private void _setPower(RobotWheelPower wheelPower) {
        leftFrontMotor.setPower(wheelPower.leftFrontPower);
        rightFrontMotor.setPower(wheelPower.rightFrontPower);
        leftRearMotor.setPower(wheelPower.leftRearPower);
        rightRearMotor.setPower(wheelPower.rightRearPower);
    }

    private double _getTargetAngle(RobotPosition pos1, RobotPosition pos2) {
        return Math.atan2(pos2.Y - pos1.Y, pos2.X - pos1.X) - pos1.Theta;

    }

    private double _getCartesianDistance(RobotPosition pos1, RobotPosition pos2) {
        return Math.sqrt( Math.pow(pos2.Y - pos1.Y, 2) + Math.pow(pos2.X - pos1.X, 2));
    }

    private double _getAngularOffset(RobotPosition pos1, RobotPosition pos2) {
        return _normalizeAngle(pos2.Theta- pos1.Theta);
    }

    private double _shiftedSigmoid(double x) {
        return 1 / (1 + Math.pow(Math.E, -x)) - 0.5;
    }

    private double _scaledShiftedSigmoid(double x, double factor) {
        return factor * _shiftedSigmoid(x);

    }

}
