//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Collections;


@TeleOp(name="Manual lift", group="Linear Opmode")
public class ManualLift extends LinearOpMode {
    // Hyper-params
    private double leftInitialPosition = 0;
    private double rightInitialPosition = 0;
    private double liftMinimumPosition = 0;
    private double liftMaximumPosition = 700;
    private double max_power = 1;
    private double min_power = -0.1;

    private double liftPID_Kp = 0.0175;
    private double liftPID_Ki = 0;
    private double liftPID_Kd = 0.00025;
    private double basePower = 0.1;

    private boolean incrementedHeight = false;
    private boolean decrementedHeight = false;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ModeSwitcher heightModeSwitcher = new ModeSwitcher(new double[]{0, 200, 500, 700}, 0);
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;


    public int getPosition() {
        double leftPosition = ((leftLift.getCurrentPosition() - leftInitialPosition));
        double rightPosition = ((rightLift.getCurrentPosition() - rightInitialPosition));
        return (int) (leftPosition + rightPosition) / 2;
    }

    public int getLeftPosition() {
        return (int) ((leftLift.getCurrentPosition() - leftInitialPosition));
    }

    public int getRightPosition() {
        return (int) ((rightLift.getCurrentPosition() - rightInitialPosition));
    }

    @Override
    public void runOpMode() {

        leftLift  = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift  = hardwareMap.get(DcMotor.class, "right_lift");

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Custom PID controller
        PIDController leftLiftController = new PIDController(liftPID_Kp, liftPID_Ki, liftPID_Kd);
        PIDController rightLiftController = new PIDController(liftPID_Kp, liftPID_Ki, liftPID_Kd);

        // get initial position of motor
        leftInitialPosition = leftLift.getCurrentPosition();
        rightInitialPosition = rightLift.getCurrentPosition();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry


            // == Gamepad triggered actions. ==

            boolean shouldIncrementHeightMode = gamepad2.dpad_up;
            boolean shouldDecrementHeightMode = gamepad2.dpad_down;

            boolean shouldMove = gamepad2.a;

            double leftPower;
            double rightPower;


            if (shouldMove) {

                //setArmPower(liftController.getNextRescaledVal(rightLift.getCurrentPosition(), runtime.seconds()));

                if (shouldIncrementHeightMode && !incrementedHeight) {
                    heightModeSwitcher.increment();

                    // to prevent executing this multiple times
                    incrementedHeight = true;

                    leftLiftController.reset((int) heightModeSwitcher.getValue(), getLeftPosition(), runtime.seconds());
                    rightLiftController.reset((int) heightModeSwitcher.getValue(), getRightPosition(), runtime.seconds());
                    continue;
                } else if (shouldDecrementHeightMode && !decrementedHeight) {
                    heightModeSwitcher.decrement();

                    // to prevent executing this multiple times
                    decrementedHeight = true;

                    leftLiftController.reset((int) heightModeSwitcher.getValue(), getLeftPosition(), runtime.seconds());
                    rightLiftController.reset((int) heightModeSwitcher.getValue(), getRightPosition(), runtime.seconds());
                    continue;
                }

                // reset value once released
                if (!shouldDecrementHeightMode) {
                    decrementedHeight = false;
                }

                if (!shouldIncrementHeightMode) {
                    incrementedHeight = false;
                }


                leftPower = leftLiftController.getNextVal(getLeftPosition(), runtime.seconds());
                rightPower = rightLiftController.getNextVal(getRightPosition(), runtime.seconds());

            } else {
                leftPower = 0;
                rightPower = 0;
            }



            leftLift.setPower(Math.max(min_power, Math.min(basePower + leftPower, max_power)));
            rightLift.setPower(Math.max(min_power, Math.min(basePower + rightPower, max_power)));
            telemetry.addData("Status", "Right PID val: " + rightPower);
            telemetry.addData("Status", "Left PID val: " + leftPower);
            telemetry.addData("Status", "Target position: " + rightLiftController.targetPosition);
            telemetry.addData("Status", "Left position: " + getLeftPosition());
            telemetry.addData("Status", "Right position: " + getRightPosition());
            telemetry.addData("Status", "A: " + leftLift.getCurrentPosition());
            telemetry.addData("Status", "B: " + rightLift.getCurrentPosition());
            telemetry.addData("Status", "Left initial position: " + leftInitialPosition);
            telemetry.addData("Status", "Right initial position: " + rightInitialPosition);
            telemetry.addData("Status", "Mode switcher value: " + heightModeSwitcher.getValue());
            telemetry.addData("Status", "Mode switcher index: " + heightModeSwitcher.currentIndex);
            telemetry.addData("Status", "Mode switcher arr: " + heightModeSwitcher.optionArr);
            telemetry.addData("Status", "left P: " + leftLiftController.currentP);
            telemetry.addData("Status", "left I: " + leftLiftController.currentI);
            telemetry.addData("Status", "left D: " + leftLiftController.currentD);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
