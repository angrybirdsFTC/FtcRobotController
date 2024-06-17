package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller_movement.ArmUtils;
import org.firstinspires.ftc.teamcode.controller_movement.DetectProp;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.util.PoseStorage;

import java.util.concurrent.atomic.AtomicReference;

public abstract class MainAutonomous extends OpMode {
    final double TILE_SIZE = 23.4;
    final double STAGE_SIZE = 70.3;
    final double ROBOT_SIZE = 17;

    // Go to prop
    final double SPIKE_Y = 28; // Y distance to spike
    final double SPIKE_CENTER_Y = 29; // Y distance to spike center
    final double SPIKE_SIDE_X = 2; // How much to move to spike's side
    final double PIXEL_RELEASE_OFFSET = -0.5; // Offset when to release pixel
    double BACK_UP = 2; // Distance to go back after placing pixel

    // Backdrop
    final double FRONT_DOWN = TILE_SIZE; // Y position to move to after placing pixel on spike in front
    final double BACKDROP_POS_X = STAGE_SIZE - TILE_SIZE * 0.5 - ROBOT_SIZE / 2.5;
    final double BACKDROP_CENTER_POS_Y = TILE_SIZE * 1.2; // The backdrop's Y position from the edge of the stage
    final double BACKDROP_LEFT_OFFSET = TILE_SIZE * 0.1; // How much to move from the center of the backdrop to the left of the backdrop
    final double BACKDROP_RIGHT_OFFSET = TILE_SIZE * 0.3; // How much to move from the center of the backdrop to the right of the backdrop
    final double WAIT_BEFORE_BACKDROP = 0.5; // Time to wait before going to backdrop
    final double EXTEND_OFFSET = 1.5; // How much time to wait after starting sequence before extending arm
    final int BACKDROP_ARM_TARGET = 1700; // Arm lift target for raising arm
    final int BACKDROP_EXTEND_TARGET = 1400; // Arm extend target for raising arm
    final int ARM_SEQUENCE_TARGET = 300; // Arm lift target for lowering arm
    final double WAIT_BEFORE_RELEASE = 2; // How much time to wait before releasing pixel
    final double WAIT_AFTER_RELEASE = 0.5; // How much time to wait after releasing pixel
    final double WAIT_FOR_RAISE = 1; // How much time to wait for the arm to raise
    final double RESET_ARM_OFFSET = 0.5; // Offset when to start putting arm down

    // Parking
    final double PARKING_INTERMEDIATE_X = BACKDROP_POS_X - 10;
    final double NEAR_PARKING = TILE_SIZE * 0.12;
    final double FAR_PARKING = TILE_SIZE * 2.6;

    protected enum Alliance {
        RED,
        BLUE
    }
    protected enum InitialPosition {
        FRONT,
        REAR
    }
    protected enum Parking {
        NEAR,
        FAR
    }

    protected abstract Alliance alliance();
    protected abstract InitialPosition initialPosition();
    protected abstract Parking parking();

    private final ElapsedTime runtime = new ElapsedTime();

    DcMotor armLift;
    DcMotor armExtend;
    Servo rightGrip;
    Servo leftGrip;
    Servo rollerServo;

    AprilTagNavigation aprilTagNavigation;
    SampleMecanumDrive drive;
    DetectProp detectProp;
    DetectProp.SpikePosition spikePosition;
    Pose2d startPose;

    int armTarget;
    int extendTarget;
    double rollerTarget;
    double leftGripTarget;
    double rightGripTarget;

    ArmUtils.ExtendDirection sequenceDirection = ArmUtils.ExtendDirection.UNINITIALIZED;
    boolean sequenceGotToPosition = false;
    int prevExtendTarget = 0;
    boolean temp = false;

    double advanceToZero(double pos, double distance) {
        return pos - Math.signum(pos) * distance;
    }

    double getStageEdge() {
        return alliance() == Alliance.BLUE ? STAGE_SIZE : -STAGE_SIZE;
    }

    void runAutonomous() {
        // Go to prop
        double spikePosX = startPose.getX();
        double spikePosY = startPose.getY();
        double spikeRot = startPose.getHeading();

        if (spikePosition == DetectProp.SpikePosition.CENTER) {
            spikePosY = advanceToZero(spikePosY, SPIKE_CENTER_Y);
        }
        else if (spikePosition == DetectProp.SpikePosition.LEFT) {
            spikePosY = advanceToZero(spikePosY, SPIKE_Y);

            if (alliance() == Alliance.BLUE) {
                spikePosX += SPIKE_SIDE_X;
                spikeRot = Math.toRadians(0);
            }
            else if (alliance() == Alliance.RED) {
                spikePosX -= SPIKE_SIDE_X;
                spikeRot = Math.toRadians(180);
            }
        }
        else if (spikePosition == DetectProp.SpikePosition.RIGHT) {
            spikePosY = advanceToZero(spikePosY, SPIKE_Y);

            if (alliance() == Alliance.BLUE) {
                spikePosX -= SPIKE_SIDE_X;
                spikeRot = Math.toRadians(180);
            }
            else if (alliance() == Alliance.RED) {
                spikePosX += SPIKE_SIDE_X;
                spikeRot = Math.toRadians(0);
            }
        }

        // Go to backdrop
//        double backdropPosY = advanceToZero(getStageEdge(), BACKDROP_CENTER_POS_Y);;
//        if (spikePosition == DetectProp.SpikePosition.LEFT) {
//            backdropPosY += BACKDROP_LEFT_OFFSET;
//        }
//        else if (spikePosition == DetectProp.SpikePosition.RIGHT) {
//            backdropPosY -= BACKDROP_RIGHT_OFFSET;
//        }
//
//        Vector2d backdropPose = new Vector2d(BACKDROP_POS_X, backdropPosY);

        // Calculate AprilTag ID
        int tagId = 0;
        switch (spikePosition) {
            case LEFT:
                tagId = 1;
                break;
            case CENTER:
                tagId = 2;
                break;
            case RIGHT:
                tagId = 3;
                break;
        }

        if (alliance() == Alliance.RED) tagId += 3;

        int finalTagId = tagId;

        double lookAtAprilTag = alliance() == Alliance.BLUE ? 45 : 315;

        // Calculate AprilTag position
        AtomicReference<Pose2d> aprilTagPose = null;

        // Something?
        double spikeCenterX = startPose.getX();
        double spikeCenterY = advanceToZero(getStageEdge(), BACKDROP_CENTER_POS_Y);

        double rearIntermediateY = advanceToZero(getStageEdge(), TILE_SIZE * 2.7);

        // Park
        double parkIntermediateY = advanceToZero(getStageEdge(), parking() == Parking.NEAR ? NEAR_PARKING : FAR_PARKING);
        double parkX = STAGE_SIZE - TILE_SIZE / 2;

        double beforeBackdropY = initialPosition() == InitialPosition.FRONT ? advanceToZero(getStageEdge(), FRONT_DOWN) : rearIntermediateY;
        double beforeBackdropX1;
        if (spikePosition == DetectProp.SpikePosition.CENTER) {
            beforeBackdropX1 = initialPosition() == InitialPosition.FRONT ? spikeCenterX + 0.1 : spikeCenterX - TILE_SIZE * 0.8; // + 0.1 for no EmptyPathSegmentException
            BACK_UP = 6;
        }
        else {
            beforeBackdropX1 = spikeCenterX + 0.1;
        }
        double beforeBackdropX2 = initialPosition() == InitialPosition.FRONT ? spikeCenterX + 0.1 : 0; // + 0.1 for no EmptyPathSegmentException

        // Build trajectory sequence
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(spikeCenterX, spikeCenterY)) // Go to spike center
                .lineTo(new Vector2d(spikePosX, spikePosY)) // Go to specific position on spike
                .lineToLinearHeading(new Pose2d(spikePosX + 0.01, spikePosY + 0.01, spikeRot)) // Rotate to spike
                .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> {
                    resetSequence();
                    armTarget = 0;
                    extendTarget = 0;
                    rollerTarget = ArmUtils.ROLLER_UPSIDEDOWN;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = 0.8;
                }) // Release pixel
                .back(BACK_UP) // Move back
                .addTemporalMarker(() -> {
                    resetSequence();
                    armTarget = 100;
                    extendTarget = 0;
                    rollerTarget = ArmUtils.ROLLER_UPSIDEDOWN;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_OPEN;
                }) // Raise arm
                .waitSeconds(WAIT_BEFORE_BACKDROP)
                .lineTo(new Vector2d(beforeBackdropX1, spikePosY)) // Nothing (front) Go towards back (rear)
                .lineTo(new Vector2d(beforeBackdropX1, beforeBackdropY)) // Go up (front) Go forward (rear)
                .lineToLinearHeading(new Pose2d(beforeBackdropX1 + 0.1, beforeBackdropY + 0.1, Math.toRadians(0.00))) // Rotate to 0
                //.lineTo(new Vector2d(beforeBackdropX2, beforeBackdropY)) // Go to center (rear)
                .lineToLinearHeading(new Pose2d(beforeBackdropX2, beforeBackdropY, Math.toRadians(lookAtAprilTag))) // Go to center
                .addTemporalMarker(() -> {
                    aprilTagPose.set(aprilTagNavigation.GetAprilTagPose(finalTagId, 5));
                })
                .addTemporalMarker(() -> {
                    resetSequence();
                    armTarget = BACKDROP_ARM_TARGET;
                    extendTarget = 0;
                    rollerTarget = ArmUtils.ROLLER_UPSIDEDOWN;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Prepare arm for backdrop
                .UNSTABLE_addTemporalMarkerOffset(EXTEND_OFFSET, () -> {
                    resetSequence();
                    armTarget = BACKDROP_ARM_TARGET;
                    extendTarget = BACKDROP_EXTEND_TARGET;
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Extend arm
                //.splineToConstantHeading(backdropPose, Math.toRadians(0.00)) // Go to backdrop
                .splineTo(new Vector2d(aprilTagPose.get().getX(), aprilTagPose.get().getY()), Math.toRadians(0)) // Go to apriltag
                .addTemporalMarker(() -> {
                    resetSequence();
                    armTarget = ARM_SEQUENCE_TARGET;
                    extendTarget = BACKDROP_EXTEND_TARGET;
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Lower arm
                .waitSeconds(WAIT_BEFORE_RELEASE)
                .addTemporalMarker(() -> {
                    resetSequence();
                    armTarget = ARM_SEQUENCE_TARGET;
                    extendTarget = BACKDROP_EXTEND_TARGET;
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_OPEN;
                    rightGripTarget = ArmUtils.GRIP_OPEN;
                }) // Release pixel
                .waitSeconds(WAIT_AFTER_RELEASE)
                .addTemporalMarker(() -> {
                    resetSequence();
                    armTarget = ARM_SEQUENCE_TARGET + 200;
                    extendTarget = BACKDROP_EXTEND_TARGET;
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_OPEN;
                    rightGripTarget = ArmUtils.GRIP_OPEN;
                }) // Close arm
                //.lineTo(new Vector2d(PARKING_INTERMEDIATE_X, backdropPose.getY())) // Go back
                .lineTo(new Vector2d(PARKING_INTERMEDIATE_X, aprilTagPose.get().getY())) // Go back
                .waitSeconds(WAIT_FOR_RAISE)
                .addTemporalMarker(() -> {
                    resetSequence();
                    temp = true;
                    armTarget = -200;
                    extendTarget = ArmUtils.ARM_EXTEND_MIN_LIMIT;
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Close arm
                .lineTo(new Vector2d(PARKING_INTERMEDIATE_X, parkIntermediateY)) // Strafe to the side
                .lineToLinearHeading(new Pose2d(PARKING_INTERMEDIATE_X + 0.1, parkIntermediateY + 0.1, Math.toRadians(180.00))) // Rotate to 180
                .lineTo(new Vector2d(parkX, parkIntermediateY)) // Park
                .build();

        // Run trajectory sequence
        drive.setPoseEstimate(trajectorySequence.start());
        drive.followTrajectorySequenceAsync(trajectorySequence);

        PoseStorage.currentPose = trajectorySequence.end();
    }

    void runSequence() {
        if (armTarget == 0 && extendTarget == 0 && rollerTarget == 0 && leftGripTarget == 0 && rightGripTarget == 0) return;

        armLift.setTargetPosition(armTarget);
        rollerServo.setPosition(rollerTarget);
        leftGrip.setPosition(leftGripTarget);
        rightGrip.setPosition(rightGripTarget);

        if (temp) {
            armExtend.setPower(-0.5);
        }

        if (prevExtendTarget != extendTarget || temp) {
            if (!sequenceGotToPosition && armLift.getCurrentPosition() > 100) {
                if (-armExtend.getCurrentPosition() < extendTarget && sequenceDirection != ArmUtils.ExtendDirection.BACKWARD) {
                    sequenceDirection = ArmUtils.ExtendDirection.FORWARD;
                    armExtend.setPower(ArmUtils.ARM_EXTEND_SPEED);
                }
                else if (-armExtend.getCurrentPosition() > extendTarget && sequenceDirection != ArmUtils.ExtendDirection.FORWARD) {
                    sequenceDirection = ArmUtils.ExtendDirection.BACKWARD;
                    armExtend.setPower(-ArmUtils.ARM_EXTEND_SPEED);
                }
                else {
                    sequenceGotToPosition = true;
                }

                sequenceGotToPosition = (sequenceDirection == ArmUtils.ExtendDirection.FORWARD && -armExtend.getCurrentPosition() >= extendTarget) || (sequenceDirection == ArmUtils.ExtendDirection.BACKWARD && -armExtend.getCurrentPosition() <= extendTarget);
            }
            else {
                armExtend.setPower(0);
            }
        }
        else {
            sequenceGotToPosition = true;
        }

        if (sequenceGotToPosition) {
            prevExtendTarget = extendTarget;
            resetSequence();
        }
    }

    void resetSequence() {
        sequenceDirection = ArmUtils.ExtendDirection.UNINITIALIZED;
        sequenceGotToPosition = false;

        armTarget = 0;
        extendTarget = 0;
        rollerTarget = 0;
        leftGripTarget = 0;
        rightGripTarget = 0;

        armExtend.setPower(0);
        armLift.setPower(ArmUtils.ARM_LIFT_POWER);
    }

    @Override
    public void init() {
        // Initialize
        drive = new SampleMecanumDrive(hardwareMap);
        detectProp = new DetectProp(hardwareMap);
        aprilTagNavigation = new AprilTagNavigation(hardwareMap, drive);

        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("gripR");
        leftGrip = hardwareMap.servo.get("gripL");
        rollerServo = hardwareMap.servo.get("roll");

        rightGrip.setDirection(Servo.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(ArmUtils.ARM_LIFT_POWER);

        leftGrip.setPosition(ArmUtils.GRIP_CLOSED);
        rightGrip.setPosition(ArmUtils.GRIP_CLOSED);

        // Calculate starting position
        double startingX = 0;
        double startingY = 0;
        double startingRotation = 0;

        if (initialPosition() == InitialPosition.FRONT) {
            startingX = TILE_SIZE / 2;
        }
        else if (initialPosition() == InitialPosition.REAR) {
            startingX = -(TILE_SIZE * 1.53);
        }

        if (alliance() == Alliance.BLUE) {
            startingY = STAGE_SIZE - ROBOT_SIZE / 2;
            startingRotation = Math.toRadians(270);
        }
        else if (alliance() == Alliance.RED) {
            startingY = -STAGE_SIZE + ROBOT_SIZE / 2;
            startingRotation = Math.toRadians(90);
        }

        startPose = new Pose2d(startingX, startingY, startingRotation);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        spikePosition = detectProp.getSpikePosition(alliance() == Alliance.BLUE ? "blueprop" : "redprop");
        telemetry.addData("size", detectProp.telemetryTfod());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();

        runAutonomous();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.update();
        runSequence();
    }
}