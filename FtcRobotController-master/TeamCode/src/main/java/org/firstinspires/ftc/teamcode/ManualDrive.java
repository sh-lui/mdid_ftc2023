package org.firstinspires.ftc.teamcode;

import android.service.autofill.DateValueSanitizer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utils.RobotWheelPower;
import org.firstinspires.ftc.teamcode.utils.RobotPosition;


@TeleOp(name="Manual drive.", group="Linear Opmode")
public class ManualDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor encoder1 = null;
    private DcMotor encoder2 = null;
    private DcMotor encoder3 = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // define the components:
        DriveBase driveBase = new DriveBase(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, encoder1, encoder2, encoder3);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // gamepad triggers
            double x =  gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn =  gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            // programming logic
            driveBase.manualSetAngularPower(theta, 0.1 * power, 0.1 * turn);

            // running components
            driveBase.run(runtime.seconds());


            // Show the elapsed game time and wheel power.
            RobotPosition currentPosition = driveBase.odometryEngine.getCurrentPosition();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Position: X: " +  currentPosition.X + " Y: " + currentPosition.Y + " Theta: " + currentPosition.Theta);
            telemetry.addData("Status", "e1: " + encoder1.getCurrentPosition());
            telemetry.addData("Status", "e2: " + encoder2.getCurrentPosition());
            telemetry.addData("Status", "e3: " + encoder3.getCurrentPosition());

            telemetry.addData("Status", "dx: " + driveBase.odometryEngine.deltaX);
            telemetry.addData("Status", "dy: " + driveBase.odometryEngine.deltaY);
            telemetry.addData("Status", "dt: " + driveBase.odometryEngine.deltaTheta);

            telemetry.addData("Status", "turn: " + turn);
            telemetry.addData("Status", "theta: " + theta);
            telemetry.addData("Status", "power: " + power);


            telemetry.update();
        }
    }}

