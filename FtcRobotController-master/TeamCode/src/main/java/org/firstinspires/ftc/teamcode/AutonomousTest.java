//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.CascadeLift;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotPosition;


@Autonomous(name="Autonomous Text.")
public class AutonomousTest extends LinearOpMode {

    private double dunkHeight = 4000; // in motor encoder ticks
    private double stackedConeOffsetHeight = 179.41117; // in motor encoder ticks

    // Declare OpMode members for each of the 4 motors.
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


    public void syncRunToPosition(RobotPosition targetPosition) {
        driveBase.setTarget(targetPosition, runtime.seconds());
        while (opModeIsActive() && driveBase.targetPosition != null) {
            driveBase.run(runtime.seconds());
        }
    }
    public void lowerAndGrab(int coneNumber) throws InterruptedException {
        // cone number: (int) from 5 ... 1.
        RobotPosition anchorPos = new RobotPosition(900, 1420, 3.96228139852);
        RobotPosition grabPos = new RobotPosition(180, 1460, Math.PI);
        int currentTraversalStage = 0;
        RobotPosition[] traversalPath = {anchorPos, grabPos};
        driveBase.overrideTolerance(50, 0.5);
        driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
        lift.setTargetHeight((coneNumber-1) * stackedConeOffsetHeight, runtime.seconds());
        arm.prepareGrab();
        claw.open();
        while (opModeIsActive() && (!lift.targetReached() || !(driveBase.targetReached() && currentTraversalStage == traversalPath.length - 1) )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());
            if (driveBase.targetReached() &&  currentTraversalStage < traversalPath.length - 1) {
                currentTraversalStage += 1;

                // additional operations for entering new stage.
                if (currentTraversalStage == 1) {
                    driveBase.overrideTolerance(10, 0.0001);
                    driveBase.overrideTranslationalPID(0.06, 0, 0.002);
                    driveBase.overrideAngularPID(0.5, 0, 0.05);

                }

                // set the new target
                driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
            }

            telemetry.addData("Status", "x: " + driveBase.odometryEngine.getCurrentPosition().X);
            telemetry.addData("Status", "y: " + driveBase.odometryEngine.getCurrentPosition().Y);
            telemetry.addData("Status", "theta: " + driveBase.odometryEngine.getCurrentPosition().Theta);
            telemetry.addData("Status", "lift target reached: " + lift.targetReached());
            telemetry.addData("Status", "drive target reached: " + driveBase.targetReached());
            telemetry.addData("Status", "drive target reached: " + driveBase.targetReached());
            telemetry.addData("Status", "dp:" + driveBase.translationalPIDController.Kp);
            telemetry.addData("Status", "dd:" + driveBase.translationalPIDController.Kd);
            telemetry.addData("Status", "adp:" + driveBase.angularPIDController.Kp);
            telemetry.addData("Status", "add:" + driveBase.angularPIDController.Kd);
            telemetry.addData("Status", "current P:" + driveBase.translationalPIDController.currentP);
            telemetry.addData("Status", "current D:" + driveBase.translationalPIDController.currentD);
            telemetry.addData("Status", "current angular P:" + driveBase.angularPIDController.currentP);
            telemetry.addData("Status", "current angular D:" + driveBase.angularPIDController.currentD);
            telemetry.addData("Status", "current angular power:" + driveBase.currentAngularPower);
            telemetry.update();
        }

        driveBase.stopAll();
        arm.grab();
        Thread.sleep(400);
        claw.close();
        Thread.sleep(200);




    }

    public void initialDunk() throws InterruptedException {
        RobotPosition anchorPos1 = new RobotPosition(900, 1520, 3.96228139852);
        RobotPosition dunkPos1 = new RobotPosition(1025.4349, 1568.746, 3.83);

        int currentTraversalStage = 0;
        RobotPosition[] traversalPath = {anchorPos1, dunkPos1};
        driveBase.overrideTolerance(50, 0.5);
        driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
        lift.setTargetHeight(dunkHeight, runtime.seconds());
        arm.prepareDunk();
        while (opModeIsActive() && (!lift.targetReached() || !(driveBase.targetReached() && currentTraversalStage == traversalPath.length - 1) )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());

            if (driveBase.targetReached() &&  currentTraversalStage < traversalPath.length - 1) {
                currentTraversalStage += 1;

                // additional operations for entering new stage.
                if (currentTraversalStage == 1) {
                    driveBase.overrideTolerance(30, 0.08);
                    driveBase.overrideTranslationalPID(0.02, 0, 0.002);
                    driveBase.overrideAngularPID(0.5, 0, 0.05);

                }

                // set the new target
                driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
            }
            telemetry.addData("Status", "x: " + driveBase.odometryEngine.getCurrentPosition().X);
            telemetry.addData("Status", "y: " + driveBase.odometryEngine.getCurrentPosition().Y);
            telemetry.addData("Status", "theta: " + driveBase.odometryEngine.getCurrentPosition().Theta);
            telemetry.addData("Status", "lift target reached: " + lift.targetReached());
            telemetry.addData("Status", "drive target reached: " + driveBase.targetReached());
            telemetry.addData("Status", "drive target reached: " + driveBase.targetReached());
            telemetry.addData("Status", "dp:" + driveBase.translationalPIDController.Kp);
            telemetry.addData("Status", "dd:" + driveBase.translationalPIDController.Kd);
            telemetry.addData("Status", "adp:" + driveBase.angularPIDController.Kp);
            telemetry.addData("Status", "add:" + driveBase.angularPIDController.Kd);
            telemetry.addData("Status", "current P:" + driveBase.translationalPIDController.currentP);
            telemetry.addData("Status", "current D:" + driveBase.translationalPIDController.currentD);
            telemetry.addData("Status", "current angular P:" + driveBase.angularPIDController.currentP);
            telemetry.addData("Status", "current angular D:" + driveBase.angularPIDController.currentD);
            telemetry.addData("Status", "current angular power:" + driveBase.currentAngularPower);
            telemetry.update();
        }

        driveBase.stopAll();

        arm.dunk();
        Thread.sleep(400);
        claw.open();
        Thread.sleep(200);
    }


    @Override
    public void runOpMode() throws InterruptedException {

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

        arm.run(0);
        claw.open();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        claw.close();


        if (opModeIsActive()) {
            initialDunk();
        }
    }
}
