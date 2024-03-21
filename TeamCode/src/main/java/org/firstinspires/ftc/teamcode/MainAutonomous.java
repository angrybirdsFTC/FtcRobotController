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
    final double SPIKE_Y = 32;
    final double SPIKE_SIDE_X = 3;
    final double PIXEL_RELEASE_OFFSET = -0.1;

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

        TrajectorySequence trajectorySequence;
        if (initialPosition() == InitialPosition.FRONT) {
            trajectorySequence = drive.trajectorySequenceBuilder(startingPosition)
                    .lineTo(new Vector2d(spikeCenterX, backdropPosY))
                    .splineTo(new Vector2d(spikePosX, spikePosY), spikeRot)
                    .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> rightGrip.setPosition(ArmUtils.GRIP_OPEN))
                    .waitSeconds(1)
                    .lineToLinearHeading(backdropPose)
                    .build();
        }
        else {
            trajectorySequence = drive.trajectorySequenceBuilder(startingPosition)
                    .lineTo(new Vector2d(spikeCenterX, backdropPosY))
                    .splineTo(new Vector2d(spikePosX, spikePosY), spikeRot)
                    .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> rightGrip.setPosition(ArmUtils.GRIP_OPEN))
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(spikeCenterX, backdropPosY, Math.toRadians(0.00)))
                    .lineToLinearHeading(backdropPose)
                    .build();
        }

        drive.setPoseEstimate(trajectorySequence.start());
        drive.followTrajectorySequence(trajectorySequence);
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
    }
}