package org.firstinspires.ftc.teamcode.controller_movement;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.PoseStorage;

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

    public MovementUtils(HardwareMap hardwareMap) {
        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        drive.setPoseEstimate(PoseStorage.currentPose);
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
}
