package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@TeleOp(name="Manual claw test", group="Linear Opmode")
public class ClawTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftServo = null;
    private Servo rightServo = null;

    private boolean hasToggledHeight = false;

    @Override
    public void runOpMode() {

        leftServo = hardwareMap.get(Servo.class, "left_claw");
        rightServo = hardwareMap.get(Servo.class, "right_claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Define the components
        Claw claw = new Claw(leftServo, rightServo);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            // receive input
            boolean clawIncrementValue = gamepad2.a;

            // logic
            if (clawIncrementValue && !hasToggledHeight) {
                claw.togglePosition();
                hasToggledHeight = true;
            }

            if (!clawIncrementValue) {
                hasToggledHeight = false;

            }

            // run components
            claw.run(runtime.seconds());


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "current position: " + claw.getCurrentPosition());
            telemetry.update();
        }

    }
}
