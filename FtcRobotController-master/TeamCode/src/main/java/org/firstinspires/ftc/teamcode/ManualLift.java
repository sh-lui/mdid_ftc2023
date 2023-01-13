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
    private double liftMinimumPosition = 10;
    private double liftRaisedPosition = 450;
    private double liftMaximumPosition = 700;

    private double liftPID_Kp = 0.007;
    private double liftPID_Ki = 0;
    private double liftPID_Kd = 0;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;


    public void setArmPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    @Override
    public void runOpMode() {

        leftLift  = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift  = hardwareMap.get(DcMotor.class, "right_lift");

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int temp = 0;

        waitForStart();
        runtime.reset();

        // Custom PID controller
        PIDController liftController = new PIDController(liftPID_Kp, liftPID_Ki, liftPID_Kd);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry


            // == Gamepad triggered actions. ==

            boolean triggerRaiseArmToUpperPosition = gamepad2.left_trigger != 0;
            boolean triggerRaiseArmToLowerPosition = gamepad2.right_bumper;
            boolean shouldMove = gamepad2.a;
            double power;



            if (shouldMove) {

                //setArmPower(liftController.getNextRescaledVal(rightLift.getCurrentPosition(), runtime.seconds()));

                if (triggerRaiseArmToUpperPosition) {
                    liftController.reset((int) liftRaisedPosition, rightLift.getCurrentPosition(), runtime.seconds());
                    temp = rightLift.getCurrentPosition();
                    continue;
                } else if (triggerRaiseArmToLowerPosition) {
                    liftController.reset((int) liftMinimumPosition, rightLift.getCurrentPosition(), runtime.seconds());
                    temp = rightLift.getCurrentPosition();
                    continue;
                } else{
                    liftController.reset(temp, rightLift.getCurrentPosition(), runtime.seconds());
                }

                power = liftController.getNextRescaledVal(rightLift.getCurrentPosition(), runtime.seconds());
                // liftController.reset(rightLift.getCurrentPosition(), rightLift.getCurrentPosition());
                // setArmPower(liftController.getNextVal());

            } else {
                power = 0;
            }



            leftLift.setPower(power);
            rightLift.setPower(power);
            telemetry.addData("Status", "PID val: " + power);
            telemetry.addData("temp: ", "value" + temp);
            telemetry.addData("Status", "Current position: " + rightLift.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
