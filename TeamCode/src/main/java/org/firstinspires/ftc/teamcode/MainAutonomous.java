package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.Roadrunner.util.PoseStorage;

public abstract class MainAutonomous extends LinearOpMode {
    final double TILE_SIZE = 23.4;
    final double STAGE_SIZE = 70.3;
    final double ROBOT_SIZE = 18;

    // Go to prop
    final double SPIKE_Y = 30; // Y distance to spike
    final double SPIKE_SIDE_X = 4; // How much to move to spike's side
    final double PIXEL_RELEASE_OFFSET = -0.75; // Offset when to release pixel
    final double BACK_UP = 5; // Distance to go back after placing pixel

    // Backdrop
    final double FRONT_DOWN = TILE_SIZE; // Y position to move to after placing pixel on spike in front
    final double BACKDROP_CENTER_POS_Y = TILE_SIZE * 1.5; // The backdrop's Y position from the edge of the stage
    final double BACKDROP_SIDE_OFFSET = TILE_SIZE * 0.25; // How much to move from the center of the backdrop to the left or right of the backdrop
    final double WAIT_BEFORE_BACKDROP = 0.5; // Time to wait before going to backdrop
    final double EXTEND_OFFSET = 1; // How much time to wait after starting sequence before extending arm
    final int ARM_SEQUENCE_TARGET = 300; // Arm lift target for lowering arm
    final double WAIT_BEFORE_RELEASE = 1; // How much time to wait before releasing pixel
    final double WAIT_AFTER_RELEASE = 0.5; // How much time to wait after releasing pixel
    final double WAIT_FOR_RAISE = 1; // How much time to wait for the arm to raise
    final double RESET_ARM_OFFSET = 0.5; // Offset when to start putting arm down

    // Parking
    final double NEAR_PARKING = TILE_SIZE * 0.5;
    final double FAR_PARKING = TILE_SIZE * 3;

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

    SampleMecanumDrive drive;
    DetectProp detectProp;

    int armTarget;
    int extendTarget;
    double rollerTarget;
    double leftGripTarget;
    double rightGripTarget;

    ArmUtils.ExtendDirection sequenceDirection = ArmUtils.ExtendDirection.UNINITIALIZED;
    boolean sequenceGotToPosition = false;

    double advanceToZero(double pos, double distance) {
        return pos - Math.signum(pos) * distance;
    }

    double getStageEdge() {
        return alliance() == Alliance.BLUE ? STAGE_SIZE : -STAGE_SIZE;
    }

    void runAutonomous(Pose2d startPose) {
        // Detect prop
        DetectProp.SpikePosition spikePosition;
        do {
            spikePosition = detectProp.getSpikePosition();
        } while (spikePosition == DetectProp.SpikePosition.NONE);

        // Go to prop
        double spikePosX = startPose.getX();
        double spikePosY = startPose.getY();
        double spikeRot = startPose.getHeading();

        if (spikePosition == DetectProp.SpikePosition.CENTER) {
            spikePosY = advanceToZero(spikePosY, SPIKE_Y);
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
        double backdropPosX = STAGE_SIZE - TILE_SIZE * 0.75;
        double backdropPosY = advanceToZero(getStageEdge(), BACKDROP_CENTER_POS_Y);;
        if (spikePosition == DetectProp.SpikePosition.LEFT) {
            backdropPosY += BACKDROP_SIDE_OFFSET;
        }
        else if (spikePosition == DetectProp.SpikePosition.RIGHT) {
            backdropPosY -= BACKDROP_SIDE_OFFSET;
        }

        Vector2d backdropPose = new Vector2d(backdropPosX, backdropPosY);

        double spikeCenterX = startPose.getX();
        double spikeCenterY = advanceToZero(getStageEdge(), BACKDROP_CENTER_POS_Y);

        double rearIntermediateY = advanceToZero(getStageEdge(), TILE_SIZE * 2.5);

        // Park
        double parkIntermediateY = advanceToZero(getStageEdge(), parking() == Parking.NEAR ? NEAR_PARKING : FAR_PARKING);
        double parkX = STAGE_SIZE - TILE_SIZE / 2;

        double beforeBackdropY = initialPosition() == InitialPosition.FRONT ? advanceToZero(getStageEdge(), FRONT_DOWN) : rearIntermediateY;
        double beforeBackdropX = initialPosition() == InitialPosition.FRONT ? spikeCenterX + 0.1 : 0; // + 0.1 for no EmptyPathSegmentException

        // Build trajectory sequence
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(spikeCenterX, spikeCenterY)) // Go to spike center
                .splineTo(new Vector2d(spikePosX, spikePosY), spikeRot) // Go to specific position on spike
                .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> rightGrip.setPosition(ArmUtils.GRIP_OPEN)) // Release pixel
                .back(BACK_UP) // Move back
                .waitSeconds(WAIT_BEFORE_BACKDROP)
                .lineToLinearHeading(new Pose2d(spikeCenterX, beforeBackdropY, Math.toRadians(0.00))) // Go down (front) Go up (rear)
                .lineTo(new Vector2d(beforeBackdropX, beforeBackdropY)) // Go to center (rear)
                .addTemporalMarker(() -> {
                    armTarget = ArmUtils.BACKDROP_ARM_TARGET;
                    extendTarget = 0;
                    rollerTarget = ArmUtils.ROLLER_UPSIDEDOWN;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Prepare arm for backdrop
                .UNSTABLE_addTemporalMarkerOffset(EXTEND_OFFSET, () -> {
                    armTarget = ArmUtils.BACKDROP_ARM_TARGET;
                    extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Extend arm
                .splineToConstantHeading(backdropPose, Math.toRadians(0.00)) // Go to backdrop
                .addTemporalMarker(() -> {
                    armTarget = ARM_SEQUENCE_TARGET;
                    extendTarget = armExtend.getCurrentPosition();
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Lower arm
                .waitSeconds(WAIT_BEFORE_RELEASE)
                .addTemporalMarker(() -> {
                    armTarget = ARM_SEQUENCE_TARGET;
                    extendTarget = armExtend.getCurrentPosition();
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_OPEN;
                    rightGripTarget = ArmUtils.GRIP_OPEN;
                }) // Release pixel
                .waitSeconds(WAIT_AFTER_RELEASE)
                .addTemporalMarker(() -> {
                    armTarget = ArmUtils.BACKDROP_ARM_TARGET;
                    extendTarget = armExtend.getCurrentPosition();
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_OPEN;
                    rightGripTarget = ArmUtils.GRIP_OPEN;
                }) // Raise arm
                .waitSeconds(WAIT_FOR_RAISE)
                .UNSTABLE_addTemporalMarkerOffset(RESET_ARM_OFFSET, () -> {
                    armTarget = -200;
                    extendTarget = 0;
                    rollerTarget = ArmUtils.ROLLER_FLAT;
                    leftGripTarget = ArmUtils.GRIP_CLOSED;
                    rightGripTarget = ArmUtils.GRIP_CLOSED;
                }) // Put down arm
                .lineTo(new Vector2d(backdropPosX, parkIntermediateY)) // Strafe to the side
                .lineToLinearHeading(new Pose2d(parkX, parkIntermediateY, Math.toRadians(180.00))) // Park
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

        if (!sequenceGotToPosition) {
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

        if (!armLift.isBusy() && sequenceGotToPosition) {
            sequenceDirection = ArmUtils.ExtendDirection.UNINITIALIZED;
            sequenceGotToPosition = false;

            armTarget = 0;
            extendTarget = 0;
            rollerTarget = 0;
            leftGripTarget = 0;
            rightGripTarget = 0;

            armExtend.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        // Initialize
        drive = new SampleMecanumDrive(hardwareMap);
        detectProp = new DetectProp(hardwareMap);

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
            startingX = -(TILE_SIZE * 1.5);
        }

        if (alliance() == Alliance.BLUE) {
            startingY = STAGE_SIZE - ROBOT_SIZE / 2;
            startingRotation = Math.toRadians(270);
        }
        else if (alliance() == Alliance.RED) {
            startingY = -STAGE_SIZE + ROBOT_SIZE / 2;
            startingRotation = Math.toRadians(90);
        }

        Pose2d startPose = new Pose2d(startingX, startingY, startingRotation);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Running");
        telemetry.update();

        runAutonomous(startPose);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.update();
            runSequence();
        }
    }
}