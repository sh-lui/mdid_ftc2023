package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Manual lift test", group="Linear Opmode")
public class ManualLiftAngleTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    public void raiseArm() {
        telemetry.addData("Status", "Raising arm");
        leftLift.setPower(1);
        rightLift.setPower(1);
    }

    public void lowerArm() {
        telemetry.addData("Status", "Lowering arm");
        leftLift.setPower(-1);
        rightLift.setPower(-1);
    }

    @Override
    public void runOpMode() {

        leftLift  = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift  = hardwareMap.get(DcMotor.class, "right_lift");

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);


        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            double leftTriggerValue = gamepad2.left_trigger;
            double rightTriggerValue = gamepad2.right_trigger;
            double power;

            if (leftTriggerValue != 0) {
                power = leftTriggerValue;
            } else {
                power = -rightTriggerValue;
            }

            leftLift.setPower(-power);
            rightLift.setPower(-power);



            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            //leftFrontDrive.setPower(leftPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Trigger Power","Power: " + power);
            telemetry.addData("Trigger Power","lt: " + leftTriggerValue);
            telemetry.addData("Trigger Power","rt: " + rightTriggerValue);
            telemetry.addData("Motor position l", "Position: " + leftLift.getCurrentPosition());
            telemetry.addData("Motor position r", "Position: " + rightLift.getCurrentPosition());
            telemetry.update();
        }

    }
}
