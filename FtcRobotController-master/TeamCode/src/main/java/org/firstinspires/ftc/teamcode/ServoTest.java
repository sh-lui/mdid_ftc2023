package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Manual servo test", group="Linear Opmode")
public class ServoTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private double MIN_POSITION = 0;
    private double MAX_POSITION = 0.65;
    private double currentPosition = 0;
    private double armRaiseRate = 0.004;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftArm = null;
    private Servo rightArm = null;

    public double incrementPosition(double currentValue, double triggerValue) {
        return Math.max(MIN_POSITION, Math.min( currentPosition + triggerValue * armRaiseRate, MAX_POSITION));
    }
    @Override
    public void runOpMode() {

        leftArm = hardwareMap.get(Servo.class, "left_arm");
        rightArm = hardwareMap.get(Servo.class, "right_arm");



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            double armIncrementValue = gamepad2.right_stick_y;
            //double rightTriggerValue = gamepad2.right_trigger;

            currentPosition = incrementPosition(currentPosition, armIncrementValue);

            leftArm.setPosition(currentPosition);
            rightArm.setPosition(1 - currentPosition);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "increment power: " + armIncrementValue);
            telemetry.addData("Status", "current position: " + currentPosition);
            telemetry.addData("Motor position l", "Position: " + leftArm.getPosition());
            telemetry.addData("Motor position r", "Position: " + rightArm.getPosition());
            telemetry.update();
        }

    }
}
