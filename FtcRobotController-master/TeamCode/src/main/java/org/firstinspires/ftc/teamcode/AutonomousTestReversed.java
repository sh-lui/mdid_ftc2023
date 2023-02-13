//package org.firstinspires.ftc.robotcontroller.external.samples;

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.CascadeLift;
import org.firstinspires.ftc.teamcode.components.Claw;
import org.firstinspires.ftc.teamcode.components.DriveBase;
import org.firstinspires.ftc.teamcode.utils.ModeSwitcher;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotPosition;
import org.firstinspires.ftc.teamcode.vision.ConeFaceRecognitionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Autonomous Test reversed (right).")
public class AutonomousTestReversed extends LinearOpMode {

    private double dunkHeight = 2775;// - 85 + 45; // in motor encoder ticks
    private double stackedConeOffsetHeight = 120.49; // in motor encoder ticks
    private double safetyDuration = 3;
    private double newDunkMinDuration = 5;

    ConeFaceRecognitionPipeline visionPipeline = new ConeFaceRecognitionPipeline();
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
    public void safetyFallback() throws InterruptedException {
        claw.open();
        Thread.sleep(100);
        arm.prepareGrab();
        Thread.sleep(300);

        lift.setTargetHeight(0, runtime.seconds());
        while (opModeIsActive() && !lift.targetReached()) {
            lift.run(runtime.seconds());
        }
    }
    public void returnBlock1() throws InterruptedException {
        RobotPosition block2Pos = new RobotPosition(-300, 1500, Math.PI - Math.PI * 3/2);

        driveBase.setTarget(block2Pos, runtime.seconds());
        driveBase.overrideTranslationalPID(0.0045, 0, 0.000005);
        driveBase.overrideAngularPID(0, 0, 0);
        driveBase.overrideTolerance(50,  2 * Math.PI);
        driveBase.overrideTranslationalCap(0.6);
        driveBase.overrideAngularCap(0.6);
        arm.prepareGrab();
        Thread.sleep(100);

        lift.setTargetHeight(0, runtime.seconds());
        while (opModeIsActive() && (!lift.targetReached() || !driveBase.targetReached() )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());
        }
    }
    public void returnBlock3() throws InterruptedException {
        RobotPosition block2Pos = new RobotPosition(-1500, 1500, Math.PI - Math.PI * 3/2);

        driveBase.setTarget(block2Pos, runtime.seconds());
        driveBase.overrideTranslationalPID(0.0045, 0, 0.000005);
        driveBase.overrideAngularPID(0, 0, 0);
        driveBase.overrideTolerance(50, 2 * Math.PI);
        driveBase.overrideTranslationalCap(0.6);
        driveBase.overrideAngularCap(0.6);
        arm.prepareGrab();
        Thread.sleep(100);

        lift.setTargetHeight(0, runtime.seconds());
        while (opModeIsActive() && (!lift.targetReached() || !driveBase.targetReached() )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());
        }
    }

    public void returnBlock2() throws InterruptedException {
        RobotPosition block2Pos = new RobotPosition(-900 , 1500, Math.PI - Math.PI);

        driveBase.setTarget(block2Pos, runtime.seconds());
        driveBase.overrideTranslationalPID(0.0045, 0, 0.000005);
        driveBase.overrideAngularPID(0, 0, 0);
        driveBase.overrideTolerance(50, 2 * Math.PI);
        driveBase.overrideTranslationalCap(0.6);
        driveBase.overrideAngularCap(0.6);
        arm.prepareGrab();
        Thread.sleep(100);

        lift.setTargetHeight(0, runtime.seconds());
        while (opModeIsActive() && (!lift.targetReached() || !driveBase.targetReached() )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());
        }
    }
    public void intermediateDunk(int coneNumber) throws InterruptedException {
        // for fixing encoder error
        double encoderOffsetX = 4;
        double encoderOffsetY = 4.5;
        int n = (5 + 1) - coneNumber;

        RobotPosition anchorPos1 = new RobotPosition(-900 , 1400, Math.PI - Math.PI);
        RobotPosition dunkPos1 = new RobotPosition(-945 - 10 - 30 + n * encoderOffsetX, 1657 - 10 + 20 + 30 + n * encoderOffsetY,  Math.PI - 3.587);

        int currentTraversalStage = 0;
        RobotPosition[] traversalPath = {anchorPos1, dunkPos1};
        driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
        driveBase.overrideTranslationalPID(0.0045, 0, 0.000005);
        driveBase.overrideAngularPID(1.5, 0, 0.005);
        driveBase.overrideTolerance(100, 0.5);
        driveBase.overrideTranslationalCap(0.6);
        driveBase.overrideAngularCap(0.6);

        lift.setTargetHeight(dunkHeight, runtime.seconds());
        arm.prepareDunk();
        while (opModeIsActive() && (!lift.targetReached() || !(driveBase.targetReached() && currentTraversalStage == traversalPath.length - 1) )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());

            if (driveBase.targetReached() &&  currentTraversalStage < traversalPath.length - 1) {
                currentTraversalStage += 1;

                // additional operations for entering new stage.
                if (currentTraversalStage == 1) {
                    driveBase.overrideTolerance(7, 0.07);
                    driveBase.overrideTranslationalPID(0.02, 0, 0.0002);
                    driveBase.overrideAngularPID(1.4, 0, 0.05);
                    driveBase.overrideTranslationalCap(0.5);
                    driveBase.overrideAngularCap(0.5);

                }

                // set the new target
                driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
            }
            if (30 - runtime.seconds() <= safetyDuration) {
                safetyFallback();
                break;
            }
            telemetry.addData("Status", "stage: intermediate dunk");
            telemetry.addData("Status", "x: " + driveBase.odometryEngine.getCurrentPosition().X);
            telemetry.addData("Status", "y: " + driveBase.odometryEngine.getCurrentPosition().Y);
            telemetry.addData("Status", "theta: " + driveBase.odometryEngine.getCurrentPosition().Theta);
            telemetry.addData("Status", "lift target reached: " + lift.targetReached());
            telemetry.addData("Status", "drive target reached: " + driveBase.targetReached());
            telemetry.addData("Status", "drive angle reached: " + driveBase.angleReached);
            telemetry.addData("Status", "drive position reached: " + driveBase.positionReached);
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
    public void lowerAndGrab(int coneNumber) throws InterruptedException {
        // cone number: (int) from 5 ... 1.
        RobotPosition anchorPos = new RobotPosition(-900, 1450, Math.PI - Math.PI);
        RobotPosition grabPos = new RobotPosition(-190, 1500, Math.PI - Math.PI);
        int currentTraversalStage = 0;
        RobotPosition[] traversalPath = {anchorPos, grabPos};
        driveBase.overrideTolerance(50, 0.007);
        driveBase.overrideTranslationalPID(0.004, 0, 0.001);
        driveBase.overrideAngularPID(1, 0, 0.01);
        driveBase.overrideTranslationalCap(0.8);
        driveBase.overrideAngularCap(0.4);
        driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
        lift.setTargetHeight((coneNumber-1) * stackedConeOffsetHeight, runtime.seconds());
        claw.open();
        Thread.sleep(50);
        arm.prepareGrab();
        Thread.sleep(100);
        while (opModeIsActive() && (!lift.targetReached() || !(driveBase.targetReached() && currentTraversalStage == traversalPath.length - 1) )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());
            if (driveBase.targetReached() &&  currentTraversalStage < traversalPath.length - 1) {
                currentTraversalStage += 1;

                // additional operations for entering new stage.
                if (currentTraversalStage == 1) {
                    driveBase.overrideTolerance(20, 0.007);
                    driveBase.overrideTranslationalPID(0.014, 0, 0.001);
                    driveBase.overrideAngularPID(3, 0, 0.01);
                    driveBase.overrideTranslationalCap(0.6);
                    driveBase.overrideAngularCap(0.6);
                }

                // set the new target
                driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
            }

            if (30 - runtime.seconds() <= safetyDuration) {
                safetyFallback();
                break;
            }
            telemetry.addData("Status", "stage: lower and grab");
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
        Thread.sleep(500);
        claw.hyperOpen();
        Thread.sleep(100);
        claw.close();
        Thread.sleep(250);




    }

    public void initialDunk() throws InterruptedException {

        RobotPosition anchorPos1 = new RobotPosition(-900, 1650, Math.PI - Math.PI/2);
        RobotPosition anchorPos2 = new RobotPosition(-900, 1380, Math.PI - Math.PI/2);
        RobotPosition dunkPos1 = new RobotPosition(-945 - 10 - 30 + 10 , 1657 - 10 + 20 + 30 - 10, Math.PI - 3.587);

        int currentTraversalStage = 0;
        RobotPosition[] traversalPath = {anchorPos1, anchorPos2, dunkPos1};
        driveBase.overrideTolerance(90, 0.5);
        driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
        driveBase.overrideTranslationalPID(0.002, 0, 0.00005);
        driveBase.overrideAngularPID(1.0, 0, 0.05);

        driveBase.overrideTranslationalCap(0.9);
        driveBase.overrideAngularCap(0.9);

        lift.setTargetHeight(dunkHeight, runtime.seconds());
        arm.prepareDunk();
        while (opModeIsActive() && (!lift.targetReached() || !(driveBase.targetReached() && currentTraversalStage == traversalPath.length - 1) )) {
            driveBase.run(runtime.seconds());
            lift.run(runtime.seconds());

            if (driveBase.targetReached() &&  currentTraversalStage < traversalPath.length - 1) {
                currentTraversalStage += 1;

                // additional operations for entering new stage.
                if (currentTraversalStage == 1) {
                    driveBase.overrideTranslationalCap(0.6);
                    driveBase.overrideAngularCap(0.6);
                }

                if (currentTraversalStage == 2) {
                    driveBase.overrideTolerance(6, 0.07);
                    driveBase.overrideTranslationalPID(0.03, 0, 0.002);
                    driveBase.overrideAngularPID(2, 0, 0.05);
                    driveBase.overrideTranslationalCap(0.3);
                    driveBase.overrideAngularCap(0.5);

                }

                // set the new target
                driveBase.setTarget(traversalPath[currentTraversalStage], runtime.seconds());
            }
            if (30 - runtime.seconds() <= safetyDuration) {
                safetyFallback();
                break;
            }
            telemetry.addData("Status", "stage: initial dunk");
            telemetry.addData("Status", "x: " + driveBase.odometryEngine.getCurrentPosition().X);
            telemetry.addData("Status", "y: " + driveBase.odometryEngine.getCurrentPosition().Y);
            telemetry.addData("Status", "theta: " + driveBase.odometryEngine.getCurrentPosition().Theta);
            telemetry.addData("Status", "lift target reached: " + lift.targetReached());
            telemetry.addData("Status", "drive target reached: " + driveBase.targetReached());
            telemetry.addData("Status", "drive angle reached: " + driveBase.angleReached);
            telemetry.addData("Status", "drive position reached: " + driveBase.positionReached);
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
        Thread.sleep(300);
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

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        Thread.sleep(400);
        claw.close();


        // initialize the cameras and connect to the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // open the camera asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                //start the streaming
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

                // execute pipeline
                camera.setPipeline(visionPipeline);

            }
            @Override
            public void onError(int errorCode) { }
        });





        // Wait for the game to start (driver presses PLAY)
        driveBase.mirrorBasePosition();
        waitForStart();
        runtime.reset();


        if (opModeIsActive()) {
            initialDunk();
            for (int i = 5; i > 2; i--) {
                if (30 - runtime.seconds() <= newDunkMinDuration) {
                    break;
                }
                lowerAndGrab(i);
                intermediateDunk(i);
            }
            returnBlock2();

            if (visionPipeline.getDeterminedSurfaceColor() == "RED") {
                returnBlock1();
            }

            if (visionPipeline.getDeterminedSurfaceColor() == "BLUE") {
                returnBlock3();
            }
        }

    }
}
