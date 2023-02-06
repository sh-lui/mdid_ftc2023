//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.CascadeLift;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;


@TeleOp(name="Manual control", group="Linear Opmode")
public class Manual extends LinearOpMode {

    private boolean incrementedLiftHeight = false;
    private boolean decrementedLiftHeight = false;
    private boolean hasToggledClawMode = false;

    private double universalSpeedReductionFactor = 0.8;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor encoder1 = null;
    private DcMotor encoder2 = null;
    private DcMotor encoder3 = null;


    private Servo leftArm = null;
    private Servo rightArm = null;

    private Servo leftClaw = null;
    private Servo rightClaw = null;

    // components
    private CascadeLift lift;
    private DriveBase driveBase;
    private Arm arm;
    private Claw claw;


    @Override
    public void runOpMode() {

        // for the lift
        leftLift  = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift  = hardwareMap.get(DcMotor.class, "right_lift");

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // for the wheels
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        encoder1 = hardwareMap.get(DcMotor.class, "right_back_drive");
        encoder2 = hardwareMap.get(DcMotor.class, "left_back_drive");
        encoder3 = hardwareMap.get(DcMotor.class, "left_front_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // for the arm
        leftArm = hardwareMap.get(Servo.class, "left_arm");
        rightArm = hardwareMap.get(Servo.class, "right_arm");

        // for the claw
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // define the components:
        lift  = new CascadeLift(leftLift, rightLift);
        driveBase = new DriveBase(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, encoder1, encoder2, encoder3);
        arm = new Arm(leftArm, rightArm);
        claw = new Claw(leftClaw, rightClaw);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // == Gamepad triggered actions. ==
            boolean shouldIncrementHeightMode = gamepad2.dpad_up;
            boolean shouldDecrementHeightMode = gamepad2.dpad_down;
            double armIncrementValue = gamepad2.right_stick_y;
            boolean shouldToggleClaw = gamepad2.right_trigger != 0;

            double drive_x =  gamepad1.left_stick_x;
            double drive_y = -gamepad1.left_stick_y;
            double drive_turn =  gamepad1.right_stick_x;
            boolean showLowerDriveSpeed = gamepad1.left_bumper || gamepad1.right_bumper;

            double drive_theta = Math.atan2(drive_y, drive_x);
            double drive_power = Math.hypot(drive_x, drive_y);

            // === Operation logic ===
            // control the lift
            if (shouldIncrementHeightMode && !incrementedLiftHeight) {
                // to prevent executing this multiple times
                incrementedLiftHeight = true;
                lift.incrementHeightMode(runtime.seconds());
                continue;
            } else if (shouldDecrementHeightMode && !decrementedLiftHeight) {
                // to prevent executing this multiple times
                decrementedLiftHeight = true;
                lift.decrementHeightMode(runtime.seconds());
                continue;
            }
            if (!shouldDecrementHeightMode) {
                decrementedLiftHeight = false;
            }

            if (!shouldIncrementHeightMode) {
                incrementedLiftHeight = false;
            }

            // control the wheels:
            driveBase.manualSetAngularPower(drive_theta, showLowerDriveSpeed ? 0.4 * drive_power * universalSpeedReductionFactor : drive_power * universalSpeedReductionFactor, showLowerDriveSpeed ? 0.4 * drive_turn * universalSpeedReductionFactor : drive_turn * universalSpeedReductionFactor);

            // control the arm:
            arm.incrementPosition(armIncrementValue);

            // control the claw:
            if (shouldToggleClaw && !hasToggledClawMode) {
                claw.togglePosition();
                hasToggledClawMode = true;
            }
            if (!shouldToggleClaw) {
                hasToggledClawMode = false;
            }




            //  === Operate the components ===
            lift.run(runtime.seconds());
            driveBase.run(runtime.seconds());
            arm.run(runtime.seconds());
            claw.run(runtime.seconds());



            telemetry.addData("Status", "Right PID val: " + lift.rightPower);
            telemetry.addData("Status", "Left PID val: " + lift.leftPower);
            telemetry.addData("Status", "Target position: " + lift.rightPIDController.targetPosition);
            telemetry.addData("Status", "Left position: " + lift.getLeftPosition());
            telemetry.addData("Status", "Right position: " + lift.getRightPosition());
            telemetry.addData("Status", "A: " + leftLift.getCurrentPosition());
            telemetry.addData("Status", "B: " + rightLift.getCurrentPosition());
            telemetry.addData("Status", "Mode switcher value: " + lift.heightModeSwitcher.getValue());
            telemetry.addData("Status", "left P: " + lift.leftPIDController.currentP);
            telemetry.addData("Status", "left I: " + lift.leftPIDController.currentI);
            telemetry.addData("Status", "left D: " + lift.leftPIDController.currentD);
            telemetry.addData("Status", "left I: " + lift.leftPIDController);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "x: " + driveBase.odometryEngine.getCurrentPosition().X);
            telemetry.addData("Status", "y: " + driveBase.odometryEngine.getCurrentPosition().Y);
            telemetry.addData("Status", "theta: " + driveBase.odometryEngine.getCurrentPosition().Theta);
            telemetry.update();
        }
    }
}
