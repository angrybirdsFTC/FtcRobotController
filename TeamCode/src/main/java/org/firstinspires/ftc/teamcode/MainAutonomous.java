package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

public abstract class MainAutonomous extends LinearOpMode {
    final double TILE_SIZE = 23.4;
    final double STAGE_SIZE = 70.3;
    final double ROBOT_SIZE = 18;

    // Go to prop
    final double SPIKE_Y = 32; // The spike's Y position
    final double SPIKE_SIDE_X = 3; // How much to move to spike's side
    final double PIXEL_RELEASE_OFFSET = -0.1; // Offset when to release pixel

    // Backdrop
    final double WAIT_BEFORE_BACKDROP = 1; // Time to wait before going to backdrop
    final double FRONT_ARM_SEQUENCE_OFFSET = 1; // Offset when to start preparing arm for placing pixel on backdrop (front)
    final double REAR_ARM_SEQUENCE_OFFSET = 4.5; // Offset when to start preparing arm for placing pixel on backdrop (rear)
    final int ARM_SEQUENCE_TARGET = 500; // Arm lift target
    final double WAIT_BEFORE_RELEASE = 2; // How much time to wait before releasing pixel
    final double WAIT_AFTER_RELEASE = 0.5; // How much time to wait after releasing pixel
    final double RESET_ARM_OFFSET = 1; // Offset when to start putting arm down

    protected enum Alliance {
        RED,
        BLUE
    }
    protected enum InitialPosition {
        FRONT,
        REAR
    }

    protected abstract Alliance alliance();
    protected abstract InitialPosition initialPosition();

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

    void runAutonomous(Pose2d startingPosition) {
        // Detect prop
        DetectProp.SpikePosition spikePosition;
        do {
            spikePosition = detectProp.getSpikePosition();
        } while (spikePosition == DetectProp.SpikePosition.NONE);

        // Go to prop
        double spikePosX = startingPosition.getX();
        double spikePosY = startingPosition.getY();
        double spikeRot = startingPosition.getHeading();

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
        double backdropPosX = STAGE_SIZE - TILE_SIZE;
        double backdropPosY = advanceToZero(alliance() == Alliance.BLUE ? STAGE_SIZE : -STAGE_SIZE, TILE_SIZE * 1.5);
        Pose2d backdropPose = new Pose2d(backdropPosX, backdropPosY, Math.toRadians(0.00));

        double spikeCenterX = startingPosition.getX();

        // Park
        double parkIntermediateY = advanceToZero(backdropPosY, TILE_SIZE);
        double parkX = STAGE_SIZE - TILE_SIZE / 2;

        // Build trajectory sequence
        TrajectorySequence trajectorySequence;
        if (initialPosition() == InitialPosition.FRONT) {
            trajectorySequence = drive.trajectorySequenceBuilder(startingPosition)
                    .lineTo(new Vector2d(spikeCenterX, backdropPosY))
                    .splineTo(new Vector2d(spikePosX, spikePosY), spikeRot)
                    .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> rightGrip.setPosition(ArmUtils.GRIP_OPEN))
                    .waitSeconds(WAIT_BEFORE_BACKDROP)
                    .UNSTABLE_addTemporalMarkerOffset(FRONT_ARM_SEQUENCE_OFFSET, () -> {
                        armTarget = ArmUtils.BACKDROP_ARM_TARGET;
                        extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = leftGrip.getPosition();
                        rightGripTarget = rightGrip.getPosition();
                    })
                    .lineToLinearHeading(backdropPose)
                    .addDisplacementMarker(() -> {
                        armTarget = ARM_SEQUENCE_TARGET;
                        extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = leftGrip.getPosition();
                        rightGripTarget = rightGrip.getPosition();
                    })
                    .waitSeconds(WAIT_BEFORE_RELEASE)
                    .addDisplacementMarker(() -> {
                        armTarget = ARM_SEQUENCE_TARGET;
                        extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = ArmUtils.GRIP_OPEN;
                        rightGripTarget = ArmUtils.GRIP_OPEN;
                    })
                    .waitSeconds(WAIT_AFTER_RELEASE)
                    .UNSTABLE_addTemporalMarkerOffset(RESET_ARM_OFFSET, () -> {
                        armTarget = -200;
                        extendTarget = 0;
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = leftGrip.getPosition();
                        rightGripTarget = rightGrip.getPosition();
                    })
                    .lineTo(new Vector2d(backdropPosX, parkIntermediateY))
                    .lineTo(new Vector2d(parkX, parkIntermediateY))
                    .build();
        }
        else {
            trajectorySequence = drive.trajectorySequenceBuilder(startingPosition)
                    .lineTo(new Vector2d(spikeCenterX, backdropPosY))
                    .splineTo(new Vector2d(spikePosX, spikePosY), spikeRot)
                    .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> rightGrip.setPosition(ArmUtils.GRIP_OPEN))
                    .waitSeconds(WAIT_BEFORE_BACKDROP)
                    .lineToLinearHeading(new Pose2d(spikeCenterX, backdropPosY, Math.toRadians(0.00)))
                    .UNSTABLE_addTemporalMarkerOffset(REAR_ARM_SEQUENCE_OFFSET, () -> {
                        armTarget = ArmUtils.BACKDROP_ARM_TARGET;
                        extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = leftGrip.getPosition();
                        rightGripTarget = rightGrip.getPosition();
                    })
                    .lineToLinearHeading(backdropPose)
                    .addDisplacementMarker(() -> {
                        armTarget = ARM_SEQUENCE_TARGET;
                        extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = leftGrip.getPosition();
                        rightGripTarget = rightGrip.getPosition();
                    })
                    .waitSeconds(WAIT_BEFORE_RELEASE)
                    .addDisplacementMarker(() -> {
                        armTarget = ARM_SEQUENCE_TARGET;
                        extendTarget = ArmUtils.BACKDROP_EXTEND_TARGET[0];
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = ArmUtils.GRIP_OPEN;
                        rightGripTarget = ArmUtils.GRIP_OPEN;
                    })
                    .waitSeconds(WAIT_AFTER_RELEASE)
                    .UNSTABLE_addTemporalMarkerOffset(RESET_ARM_OFFSET, () -> {
                        armTarget = -200;
                        extendTarget = 0;
                        rollerTarget = ArmUtils.ROLLER_FLAT;
                        leftGripTarget = leftGrip.getPosition();
                        rightGripTarget = rightGrip.getPosition();
                    })
                    .lineTo(new Vector2d(backdropPosX, parkIntermediateY))
                    .lineTo(new Vector2d(parkX, parkIntermediateY))
                    .build();
        }

        // Run trajectory sequence
        drive.setPoseEstimate(trajectorySequence.start());
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    void runSequence() {
        if (armTarget == 0 && extendTarget == 0 && rollerTarget == 0 && leftGripTarget == 0 && rightGripTarget == 0) return;

        armLift.setTargetPosition(armTarget);
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

            sequenceGotToPosition = (sequenceDirection == ArmUtils.ExtendDirection.FORWARD && -armExtend.getCurrentPosition() >= extendTarget) || (sequenceDirection == ArmUtils.ExtendDirection.BACKWARD && -armExtend.getCurrentPosition() <= extendTarget);
        }
        else {
            armExtend.setPower(0);
        }

        if (!armLift.isBusy() && sequenceGotToPosition) {
            rollerServo.setPosition(rollerTarget);

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
            runSequence();
        }
    }
}