package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class CascadeLift extends BaseComponent{
    private double leftInitialPosition = 0;
    private double rightInitialPosition = 0;
    private double max_power = 1;
    private double min_power = -0.05;

    private double liftPID_Kp = 0.04;
    private double liftPID_Ki = 0;
    private double liftPID_Kd = 0.00005;
    private double liftPID_IErrorThres = 200;
    private double basePower = 0.1;

    // variables set public for debugging purposes
    public ModeSwitcher heightModeSwitcher = new ModeSwitcher(new double[]{0, 200, 470, 720}, 0);
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    public PIDController leftPIDController;
    public PIDController rightPIDController;

    public double leftPower;
    public double rightPower;



    public CascadeLift(DcMotor _leftMotor, DcMotor _rightMotor) {
        leftMotor = _leftMotor;
        rightMotor = _rightMotor;

        // create the PID controllers
        leftPIDController = new PIDController(liftPID_Kp, liftPID_Ki, liftPID_Kd, liftPID_IErrorThres);
        rightPIDController = new PIDController(liftPID_Kp, liftPID_Ki, liftPID_Kd, liftPID_IErrorThres);

        // get initial position of motor
        leftInitialPosition = leftMotor.getCurrentPosition();
        rightInitialPosition = rightMotor.getCurrentPosition();
    }

    public int getPosition() {
        double leftPosition = ((leftMotor.getCurrentPosition() - leftInitialPosition));
        double rightPosition = ((rightMotor.getCurrentPosition() - rightInitialPosition));
        return (int) (leftPosition + rightPosition) / 2;
    }

    public int getLeftPosition() {
        return (int) ((leftMotor.getCurrentPosition() - leftInitialPosition));
    }

    public int getRightPosition() {
        return (int) ((rightMotor.getCurrentPosition() - rightInitialPosition));
    }

    public void setTargetHeight(double value, double time) {

        leftPIDController.reset(value, getLeftPosition(), time);
        rightPIDController.reset(value, getRightPosition(), time);
    }

    public void incrementHeightMode(double time) {
        heightModeSwitcher.increment();
        setTargetHeight(heightModeSwitcher.getValue(), time);
    }

    public void decrementHeightMode(double time) {
        heightModeSwitcher.decrement();
        setTargetHeight(heightModeSwitcher.getValue(), time);
    }

    public void run(double time) {

        leftPower = leftPIDController.getNextVal(getLeftPosition(), time);
        rightPower = rightPIDController.getNextVal(getRightPosition(), time);

        leftMotor.setPower(Math.max(min_power, Math.min(basePower + leftPower, max_power)));
        rightMotor.setPower(Math.max(min_power, Math.min(basePower + rightPower, max_power)));

    }
}
