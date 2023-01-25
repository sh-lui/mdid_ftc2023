//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.components.CascadeLift;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotPosition;


@Autonomous(name="Autonomous Text.")
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

        encoder1 = hardwareMap.get(DcMotor.class, "right_back_drive");
        encoder2 = hardwareMap.get(DcMotor.class, "left_back_drive");
        encoder3 = hardwareMap.get(DcMotor.class, "left_front_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // define the components:
        DriveBase driveBase = new DriveBase(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, encoder1, encoder2, encoder3);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        RobotPosition robotTarget = new RobotPosition(0, 1000, Math.PI);
        driveBase.setTarget(robotTarget, runtime.seconds());

        while (opModeIsActive()) {

            // === Operation logic ===

            //  === Operate the components ===
            driveBase.run(runtime.seconds());


            RobotPosition currentPosition = driveBase.odometryEngine.getCurrentPosition();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "X: " + currentPosition.X);
            telemetry.addData("Status", "Y: "  + currentPosition.Y);
            telemetry.addData("Status", "Theta: " + currentPosition.Theta);

            telemetry.addData("Status", "Target X: " + driveBase.targetPosition.X);
            telemetry.addData("Status", "Target Y: " + driveBase.targetPosition.Y);
            telemetry.addData("Status", "Target Theta: " + driveBase.targetPosition.Theta);

            telemetry.addData("Status", "autoTheta: " + driveBase.autoTheta);
            telemetry.addData("Status", "autoPower: " + driveBase.autoPower);
            telemetry.addData("Status", "autoTurn: " + driveBase.autoTurn);
            telemetry.addData("Status", "angularOffset: " + driveBase.angularOffset);
            telemetry.update();
        }
    }
}
