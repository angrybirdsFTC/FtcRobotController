package org.firstinspires.ftc.teamcode.opencv;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AprilTag yellow pixels")
public class AprilTagRects extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */

    private ApriltagProcessorRects apriltagrects;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() {
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), apriltagrects);

        telemetry.update();
        // Wait for the DS start button to be touched.``
        waitForStart();

        if (opModeIsActive()) {
            //..
        }
        visionPortal.close();
    }
}