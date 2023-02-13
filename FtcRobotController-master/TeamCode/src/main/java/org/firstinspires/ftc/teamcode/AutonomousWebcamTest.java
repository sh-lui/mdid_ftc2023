package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.robotcontroller.external.samples;

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

@Autonomous(name="Autonomous Webcam Text.")
public class AutonomousWebcamTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize the cameras and connect to the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        ConeFaceRecognitionPipeline visionPipeline = new ConeFaceRecognitionPipeline();

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

        // wait for start
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Result:" + visionPipeline.getDeterminedSurfaceColor());
            telemetry.addData("Status", "Result:" + visionPipeline.getMsg());
            telemetry.addData("Status", "result:" + cameraMonitorViewId  );
            telemetry.update();
        }

    }

}


