package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.RobotWheelPower;


@TeleOp(name="Manual drive.", group="Linear Opmode")
public class ManualDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;



    public static  RobotWheelPower getMotorPowerFromAngularPower(double theta, double power, double turn) {
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

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double x =  gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn =  gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);


            RobotWheelPower resultantPower = getMotorPowerFromAngularPower(theta, power, turn);

            // Send calculated power to wheels
            leftFrontDrive.setPower( resultantPower.leftFrontPower);
            rightFrontDrive.setPower(resultantPower.rightFrontPower);
            leftBackDrive.setPower(resultantPower.leftRearPower);
            rightBackDrive.setPower(resultantPower.rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", resultantPower.leftFrontPower, resultantPower.rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", resultantPower.leftRearPower, resultantPower.rightRearPower);
            telemetry.update();
        }
    }}

