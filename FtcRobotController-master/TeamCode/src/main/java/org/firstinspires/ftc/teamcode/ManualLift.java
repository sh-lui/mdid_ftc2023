//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.components.CascadeLift;
import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import org.firstinspires.ftc.teamcode.utils.PIDController;


@TeleOp(name="Manual lift", group="Linear Opmode")
public class ManualLift extends LinearOpMode {
    // Hyper-params
    private double leftInitialPosition = 0;
    private double rightInitialPosition = 0;
    private double max_power = 1;
    private double min_power = -0.1;

    private double liftPID_Kp = 0.0175;
    private double liftPID_Ki = 0;
    private double liftPID_Kd = 0.00025;
    private double liftPID_IErrorThres = 200;
    private double basePower = 0.1;

    private boolean incrementedHeight = false;
    private boolean decrementedHeight = false;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ModeSwitcher heightModeSwitcher = new ModeSwitcher(new double[]{0, 200, 500, 700}, 0);
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;


    // components
    private CascadeLift lift;


    @Override
    public void runOpMode() {

        leftLift  = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift  = hardwareMap.get(DcMotor.class, "right_lift");

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // define the components:
        lift  = new CascadeLift(leftLift, rightLift);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry


            // == Gamepad triggered actions. ==

            boolean shouldIncrementHeightMode = gamepad2.dpad_up;
            boolean shouldDecrementHeightMode = gamepad2.dpad_down;

            boolean shouldMove = gamepad2.a;




            // === trigger operations ===
            if (shouldIncrementHeightMode && !incrementedHeight) {

                // to prevent executing this multiple times
                incrementedHeight = true;
                lift.incrementHeightMode(runtime.seconds());
                continue;
            } else if (shouldDecrementHeightMode && !decrementedHeight) {

                // to prevent executing this multiple times
                decrementedHeight = true;
                lift.decrementHeightMode(runtime.seconds());
                continue;
            }

            // reset value once released
            if (!shouldDecrementHeightMode) {
                decrementedHeight = false;
            }

            if (!shouldIncrementHeightMode) {
                incrementedHeight = false;
            }

            //  === Operate the components ===
            lift.run(runtime.seconds());



            telemetry.addData("Status", "Right PID val: " + lift.rightPower);
            telemetry.addData("Status", "Left PID val: " + lift.leftPower);
            telemetry.addData("Status", "Target position: " + lift.rightPIDController.targetPosition);
            telemetry.addData("Status", "Left position: " + lift.getLeftPosition());
            telemetry.addData("Status", "Right position: " + lift.getRightPosition());
            telemetry.addData("Status", "A: " + leftLift.getCurrentPosition());
            telemetry.addData("Status", "B: " + rightLift.getCurrentPosition());
            telemetry.addData("Status", "Left initial position: " + leftInitialPosition);
            telemetry.addData("Status", "Right initial position: " + rightInitialPosition);
            telemetry.addData("Status", "Mode switcher value: " + heightModeSwitcher.getValue());
            telemetry.addData("Status", "Mode switcher index: " + heightModeSwitcher.currentIndex);
            telemetry.addData("Status", "Mode switcher arr: " + heightModeSwitcher.optionArr);
            telemetry.addData("Status", "left P: " + lift.leftPIDController.currentP);
            telemetry.addData("Status", "left I: " + lift.leftPIDController.currentI);
            telemetry.addData("Status", "left D: " + lift.leftPIDController.currentD);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
