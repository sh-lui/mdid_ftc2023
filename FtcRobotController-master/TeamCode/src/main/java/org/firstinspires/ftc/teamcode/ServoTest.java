package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp(name="Manual servo test", group="Linear Opmode")
public class ServoTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override
    public void runOpMode() {

        leftServo = hardwareMap.get(Servo.class, "left_arm");
        rightServo = hardwareMap.get(Servo.class, "right_arm");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Define the components
        Arm arm = new Arm(leftServo, rightServo);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            // receive input
            double armIncrementValue = gamepad2.right_stick_y;

            // logic
            arm.incrementPosition(armIncrementValue);

            // run components
            arm.run(runtime.seconds());


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "increment power: " + armIncrementValue);
            telemetry.addData("Status", "current position: " + arm.getCurrentPosition());
            telemetry.update();
        }

    }
}
