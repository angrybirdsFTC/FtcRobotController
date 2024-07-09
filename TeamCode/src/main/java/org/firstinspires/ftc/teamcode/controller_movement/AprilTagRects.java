package org.firstinspires.ftc.teamcode.controller_movement;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.opencv.detectyellowPixels;
import java.lang.annotation.Target;

@TeleOp(name = "AprilTag yellow pixels")
public class AprilTagRects extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */

    //private ApriltagProcessorRects apriltagrects;

    private AprilTagProcessor aprilTag;

    private OpenCvCamera Camera;

    final int CAMERA_WIDTH = 1920;
    final int CAMERA_HEIGHT = 1080;
    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        Camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Camera.setPipeline(new detectyellowPixels());
        Camera.openCameraDevice();
        Camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(Camera, 30);
        /**
         * The variable to store our instance of the vision portal.
         */


        telemetry.update();
        // Wait for the DS start button to be touched.``
        waitForStart();


        if (opModeIsActive()) {
            telemetry.addData("Status:", "Running");
            telemetry.update();
        }

        Camera.stopStreaming();
    }
}