package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class MovementUtils {
    final double AXIAL_SPEED = 1;
    final double LATERAL_SPEED = 1;
    final double YAW_SPEED = 1;
    final double SLOW_MODE_MULTIPLIER = 0.3;

    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    double axialMultiplier;
    double lateralMultiplier;
    double yawMultiplier;

    SampleMecanumDrive drive;

    PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    Vector2d targetPosition = new Vector2d(0, 0);

    public MovementUtils(HardwareMap hardwareMap) {
        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        //drive.setPoseEstimate(PoseStorage.currentPose);

        headingController.setInputBounds(-Math.PI, Math.PI);
    }

    void calculateMultipliers(Gamepad gamepad) {
        boolean slowModeActive = gamepad.right_bumper || Controller.Instance.gamepad2.start;

        axialMultiplier = AXIAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        lateralMultiplier = LATERAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        yawMultiplier = YAW_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
    }

    public void movement(Gamepad gamepad) {
        calculateMultipliers(gamepad);

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y * axialMultiplier,
                        -gamepad.left_stick_x * lateralMultiplier,
                        -gamepad.right_stick_x * yawMultiplier
                )
        );

        drive.update();
    }

    public void fieldCentricMovement(Gamepad gamepad) {
        calculateMultipliers(gamepad);

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * axialMultiplier,
                        input.getY() * lateralMultiplier,
                        -gamepad.right_stick_x * yawMultiplier
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }

    public void testMovement(Gamepad gamepad) {
        // Read pose
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection;

        // Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x
        );
        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(poseEstimate.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();

        // Not technically omega because its power. This is the derivative of atan2
        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        // Set the target heading for the heading controller to our desired angle
        headingController.setTargetPosition(theta);

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        double headingInput = (headingController.update(poseEstimate.getHeading())
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

        // Combine the field centric x/y velocity with our derived angular velocity
        driveDirection = new Pose2d(
                robotFrameInput,
                headingInput
        );

        drive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        drive.getLocalizer().update();
    }
}