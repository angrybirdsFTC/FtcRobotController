package org.firstinspires.ftc.teamcode.controller_movement;

import android.annotation.SuppressLint;
import android.icu.util.ICUUncheckedIOException;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.integration.BaseAbstractUnivariateIntegrator;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.opencv.ApriltagProcessorRects;
import org.firstinspires.ftc.teamcode.utils.Transform3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import java.util.List;
import java.math.*;

@TeleOp(name="AprilTagCorrectionTest", group = "SA_FTC")
public class AprilTagCorrectionTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */


    private AprilTagProcessor aprilTag;
    int TARGET_ID = 3;
    final double BACKDROP_SPACE_DISTANCE = 10;

    final double CAMERA_TO_MIDDLE = 10.6; // in
    double detectionX;
    double detectionY;
    double detectionYaw;
    double detectionBearing;
    double correctionX;
    double correctionY;
    AprilTagDetection detection;

    double correctionAngle;

    final int CAMERA_WIDTH = 1920;
    final int CAMERA_HEIGHT = 1080;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    SampleMecanumDrive drive;
    //MovementUtils movement = new MovementUtils(HardwareMap);


    public Pose2d calculatePose(double X, double Y, double yaw, double bearing, double range) {
        double tagX = this.detection.metadata.fieldPosition.get(0);
        double tagY = this.detection.metadata.fieldPosition.get(1);

        double theta = Math.toRadians(bearing - yaw);

        double camera_posX = tagX - range * Math.cos(theta);
        double camera_posY = tagY - range * Math.sin(theta);

        double yaw_to_rad = Math.toRadians(yaw);

        double posX = camera_posX - CAMERA_TO_MIDDLE * Math.cos(yaw_to_rad);
        double posY = camera_posY + CAMERA_TO_MIDDLE * Math.sin(yaw_to_rad);

        return new Pose2d(posX, posY, Math.toRadians(-yaw));
    }

    public Pose2d calculateCameraPose(double X, double Y, double yaw, double bearing, double range) {
        double tagX = this.detection.metadata.fieldPosition.get(0);
        double tagY = this.detection.metadata.fieldPosition.get(1);

        double theta = Math.toRadians(bearing - yaw);

        double camera_posX = tagX - range * Math.cos(theta);
        double camera_posY = tagY - range * Math.sin(theta);

        return new Pose2d(camera_posX, camera_posY, Math.toRadians(-yaw));
    }

    public Pose2d calculatePoseHopefully(double X, double Y, double yaw, double tagX, double tagY) {
        Y -= CAMERA_TO_MIDDLE;
        double theta = Math.toRadians(-yaw);

       double relativeX = X * Math.cos(theta) + Y * Math.sin(theta);
       double relativeY = Y * Math.cos(theta) - X * Math.sin(theta);

       double robotX = tagX - relativeX;
       double robotY = tagY - relativeY;

       return new Pose2d(robotX, robotY, theta);
    }

    public Pose2d calculatePose2(double X, double Y, double yaw) {
        double tagX = this.detection.metadata.fieldPosition.get(0);
        double tagY = this.detection.metadata.fieldPosition.get(1);

        double camera_posX = tagX + Y;
        double camera_posY = tagY - X;

        double yaw_to_rad = Math.toRadians(yaw);
        double posY = camera_posY - CAMERA_TO_MIDDLE * Math.sin(yaw_to_rad);
        double posX = camera_posX - CAMERA_TO_MIDDLE * Math.cos(yaw_to_rad);
        Pose2d currPos = new Pose2d(posX, posY, Math.toRadians(-yaw));

        return currPos;

//        AprilTagDetection tag = this.detection;
//
//        // Get the tag absolute position on the field
//        Transform3D tagPose = new Transform3D(
//                tag.metadata.fieldPosition,
//                tag.metadata.fieldOrientation
//        );
//
//        // Get the relative location of the tag from the camera
//        Transform3D cameraToTagTransform = new Transform3D(
//                new VectorF(
//                        (float) tag.rawPose.x,
//                        (float) tag.rawPose.y,
//                        (float) tag.rawPose.z
//                ),
//                Transform3D.MatrixToQuaternion(tag.rawPose.R)
//        );
//
//        // Inverse the previous transform to get the location of the camera from the tag
//        Transform3D tagToCameraTransform = cameraToTagTransform.unaryMinusInverse();
//
//        // Add the tag position and the relative position of the camera to the tag
//        Transform3D cameraPose = tagPose.plus(tagToCameraTransform);
//
//        // The the relative location of the camera to the robot
//        //TODO: You have to tune this value for your camera
//        Transform3D robotToCameraTransform = new Transform3D(
//                new VectorF(
//                        0f,
//                        10.5f,
//                        6f
//                ),
//                new Quaternion(0,0,1f,0, System.nanoTime())
//        );
//
//        // Inverse the previous transform again to get the location of the robot from the camera
//        Transform3D cameraToRobotTransform = robotToCameraTransform.unaryMinusInverse();
//
//        // Add the relative location of the robot to location of the Camera
//        Transform3D robotPose = cameraPose.plus(cameraToRobotTransform);
//
//        // Convert from a 3D transform to a 2D pose
//        return robotPose.toPose2d();
    }

    public void MoveWithSpline(double X, double Y, double bearing, double yaw, double range) {
        double tagX = detection.metadata.fieldPosition.get(0);
        double tagY = detection.metadata.fieldPosition.get(1);

        Pose2d currPos = calculatePoseHopefully(X, Y, yaw, tagX, tagY);

        drive.setPoseEstimate(currPos);
        Trajectory t3 = drive.trajectoryBuilder(currPos)
                .splineToLinearHeading(new Pose2d(tagX - 20, tagY , Math.toRadians(0)), Math.toRadians(0))
                .build();
        drive.followTrajectory(t3);
    }

    public void turnalittlebittotheright() {
        drive.turn(Math.toRadians(5));
    }

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag(TARGET_ID);
                // Push telemetry to the Driver Station.
                telemetry.addData("Target ID: ", TARGET_ID);
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                if (gamepad1.a) {
                    TARGET_ID = 1;
                }
                if (gamepad1.b) {
                    TARGET_ID = 2;
                }
                else if (gamepad1.y) {
                    TARGET_ID = 3;
                }

                if (gamepad1.right_bumper) {
                    // does nothing
                }
                if (gamepad1.left_bumper) {
                   turnalittlebittotheright();
                }
                if (gamepad1.guide) {
                    MoveWithSpline(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.bearing, detection.ftcPose.yaw, detection.ftcPose.range);
                }
                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                 .setLensIntrinsics(1413.91, 1413.91, 965.446, 529.378)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable git the processor.
        builder.addProcessor(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    public void telemetryAprilTag(int target_id) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == target_id) {
                this.detection = detection;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine("Distance from AprilTag: " + detection.ftcPose.y);
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                this.detectionX = detection.ftcPose.x;
                telemetry.addData("Raw pos x: ", detection.ftcPose.x);
                telemetry.addData("Raw pos y: ", detection.ftcPose.y);
                telemetry.addData("Raw pos z: ", detection.ftcPose.z);
                telemetry.addData("april tag pos: ", detection.metadata.fieldPosition);
                telemetry.addData("april tag rot: ", detection.ftcPose.yaw);
                telemetry.addData("april tag bearing: ", detection.ftcPose.bearing);
                telemetry.addData("posx camera", detection.metadata.fieldPosition.get(0) - detection.ftcPose.y);
                telemetry.addData("posy camera", detection.metadata.fieldPosition.get(1) + detection.ftcPose.x);
                telemetry.addData("fcb american patriots: ", calculatePose(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw, detection.ftcPose.bearing, detection.ftcPose.range));
                telemetry.addData("camera pose: ", calculateCameraPose(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw, detection.ftcPose.bearing, detection.ftcPose.range));
            }
        }   // end for() loop
    }
}
