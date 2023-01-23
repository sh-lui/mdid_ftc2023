//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.components.CascadeLift;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotPosition;


@TeleOp(name="Autonomous Text.", group="Linear Opmode")
public class AutonomousTest extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor encoder1 = null;
    private DcMotor encoder2 = null;
    private DcMotor encoder3 = null;



    // components
    private CascadeLift lift;


    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        encoder1 = hardwareMap.get(DcMotor.class, "encoder1");
        encoder2 = hardwareMap.get(DcMotor.class, "encoder2");
        encoder3 = hardwareMap.get(DcMotor.class, "encoder3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // define the components:
        DriveBase driveBase = new DriveBase(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, encoder1, encoder2, encoder3);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        RobotPosition robotTarget = new RobotPosition(1, 1, Math.PI);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // == Gamepad triggered actions. ==
            boolean setTarget = gamepad1.a;

            // === Operation logic ===
            if (setTarget) {
                driveBase.setTarget(robotTarget, runtime.seconds());
                continue;
            }

            //  === Operate the components ===
            driveBase.run(runtime.seconds());


            telemetry.addData("Status", "Right PID val: " + lift.rightPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
