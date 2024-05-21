package org.firstinspires.ftc.teamcode.controller_movement;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import java.util.List;

@TeleOp(name="AprilTagCorrectionTest", group = "SA_FTC")
public class AprilTagCorrectionTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */


    private AprilTagProcessor aprilTag;
    int TARGET_ID = 3;
    final double BACKDROP_SPACE_DISTANCE = 10;

    double correctionX;
    double correctionY;

    double correctionAngle;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    //MovementUtils movement = new MovementUtils(HardwareMap);

    //returns the correction
    public double correctDistanceY(double yDistance, double TargetY) {
        double correction = yDistance - TargetY;
        return Math.abs(correction);
    }

    // This is used for the Strafe movement only
    public double correctDistanceX(double xDistance, double TargetX) {
        return xDistance - TargetX;
    }

    // returns the correction regarding the robot's angle
    public double correctAngle(double AprilTagAngle, double TargetAngle) {
        return AprilTagAngle - TargetAngle;
    }

    public void MoveWithCorrectionY(double correction, MovementUtils movementUtils) {
        Trajectory t1 =  movementUtils.drive.trajectoryBuilder(new Pose2d())
                .forward(correction)
                .build();
        movementUtils.drive.followTrajectory(t1);
    }

    public void MoveWithCorrectionX(double correction, MovementUtils movementUtils) {
        Trajectory t2;

        if (correction < 0) {
            t2 = movementUtils.drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(Math.abs(correction))
                    .build();
        }

        else {
            t2 = movementUtils.drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(Math.abs(correction))
                    .build();
        }

        movementUtils.drive.followTrajectory(t2);
    }

    public void MoveWithSpline(double correctionX, double correctionY, double angle_correction, MovementUtils movementUtils) {
        Trajectory t3 = movementUtils.drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(correctionX, correctionY, Math.toRadians(angle_correction)))
                .build();
        movementUtils.drive.followTrajectory(t3);
    }
    @Override
    public void runOpMode() {
        MovementUtils movementUtils = new MovementUtils(hardwareMap);

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
                    MoveWithCorrectionX(correctionX, movementUtils);
                }
                if (gamepad1.left_bumper) {
                    MoveWithCorrectionY(correctionY, movementUtils);
                }
                if (gamepad1.guide) {
                    MoveWithSpline(-1 * correctionX,-1 * correctionY, -1 * correctionAngle, movementUtils);
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
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
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
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
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
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine("Distance from AprilTag: " + detection.ftcPose.y);
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                telemetry.addData("Forward correction: ", correctDistanceY(detection.ftcPose.y, BACKDROP_SPACE_DISTANCE));
                telemetry.addData("Angle correction: ", correctAngle(detection.ftcPose.roll, 0));
                telemetry.addData("Strafe correction: ",correctDistanceX(detection.ftcPose.x, 0));

                correctionAngle = correctAngle(detection.ftcPose.roll, 0);
                correctionX = correctDistanceX(detection.ftcPose.x, 0);
                correctionY = correctDistanceY(detection.ftcPose.y, BACKDROP_SPACE_DISTANCE);
            }
        }   // end for() loop

        // Add "key" information to telemetry
        //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
