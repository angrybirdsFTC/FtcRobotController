package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.io.Console;

public class MeepMeepTesting {
    enum Alliance {
        RED,
        BLUE
    }
    enum InitialPosition {
        FRONT,
        REAR
    }
    enum Parking {
        NEAR,
        FAR
    }
    enum SpikePosition {
        NONE,
        LEFT,
        RIGHT,
        CENTER
    }

    static Alliance alliance() {
        return Alliance.RED;
    }
    static InitialPosition initialPosition() {
        return InitialPosition.REAR;
    }
    static Parking parking() {
        return Parking.NEAR;
    }
    static SpikePosition spikePosition = SpikePosition.LEFT;

    static final double TILE_SIZE = 23.4;
    static final double STAGE_SIZE = 70.3;
    static final double ROBOT_SIZE = 18;

    // Go to prop
    static final double SPIKE_Y = 30; // Y distance to spike
    static final double SPIKE_SIDE_X = 4; // How much to move to spike's side
    static final double PIXEL_RELEASE_OFFSET = -0.75; // Offset when to release pixel
    static final double BACK_UP = 5; // Distance to go back after placing pixel

    // Backdrop
    static final double FRONT_DOWN = TILE_SIZE; // Y position to move to after placing pixel on spike in front
    static final double BACKDROP_CENTER_POS_Y = TILE_SIZE * 1.5; // The backdrop's Y position from the edge of the stage
    static final double BACKDROP_SIDE_OFFSET = TILE_SIZE * 0.25; // How much to move from the center of the backdrop to the left or right of the backdrop
    static final double WAIT_BEFORE_BACKDROP = 0.5; // Time to wait before going to backdrop
    static final double EXTEND_OFFSET = 1; // How much time to wait after starting sequence before extending arm
    static final int ARM_SEQUENCE_TARGET = 300; // Arm lift target for lowering arm
    static final double WAIT_BEFORE_RELEASE = 1; // How much time to wait before releasing pixel
    static final double WAIT_AFTER_RELEASE = 0.5; // How much time to wait after releasing pixel
    static final double WAIT_FOR_RAISE = 1; // How much time to wait for the arm to raise
    static final double RESET_ARM_OFFSET = 0.5; // Offset when to start putting arm down

    // Parking
    static final double NEAR_PARKING = TILE_SIZE * 0.5;
    static final double FAR_PARKING = TILE_SIZE * 3;

    static double advanceToZero(double pos, double distance) {
        return pos - Math.signum(pos) * distance;
    }

    static double getStageEdge() {
        return alliance() == Alliance.BLUE ? STAGE_SIZE : -STAGE_SIZE;
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
//                .setStartPose(new Pose2d(-36.33, 62.75, Math.toRadians(270.00)))
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.33, 62.75, Math.toRadians(270.00)))
//                                .splineTo(new Vector2d(-22.36, 0.85), Math.toRadians(0.47))
//                                .splineTo(new Vector2d(61.43, 0.28), Math.toRadians(0.00))
//                                .lineToConstantHeading(new Vector2d(61.43, 14.44))
//                                .build()
//                );

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

        // Go to prop
        double spikePosX = startPose.getX();
        double spikePosY = startPose.getY();
        double spikeRot = startPose.getHeading();

        if (spikePosition == SpikePosition.CENTER) {
            spikePosY = advanceToZero(spikePosY, SPIKE_Y);
        }
        else if (spikePosition == SpikePosition.LEFT) {
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
        else if (spikePosition == SpikePosition.RIGHT) {
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
        if (spikePosition == SpikePosition.LEFT) {
            backdropPosY += BACKDROP_SIDE_OFFSET;
        }
        else if (spikePosition == SpikePosition.RIGHT) {
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

        double finalSpikePosX = spikePosX;
        double finalSpikePosY = spikePosY;
        double finalSpikeRot = spikeRot;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
                .setStartPose(startPose)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(spikeCenterX, spikeCenterY)) // Go to spike center
                                .splineTo(new Vector2d(finalSpikePosX, finalSpikePosY), finalSpikeRot) // Go to specific position on spike
                                .UNSTABLE_addTemporalMarkerOffset(PIXEL_RELEASE_OFFSET, () -> System.out.println("Released pixel")) // Release pixel
                                .back(BACK_UP) // Move back
                                .waitSeconds(WAIT_BEFORE_BACKDROP)
                                .lineToLinearHeading(new Pose2d(spikeCenterX, beforeBackdropY, Math.toRadians(0.00))) // Go up (front) Go forward (rear)
                                .lineTo(new Vector2d(beforeBackdropX, beforeBackdropY)) // Go to center (rear)
                                .addTemporalMarker(() -> System.out.println("Raising arm")) // Prepare arm for backdrop
                                .UNSTABLE_addTemporalMarkerOffset(EXTEND_OFFSET, () -> System.out.println("Extending arm")) // Extend arm
                                .splineToConstantHeading(backdropPose, Math.toRadians(0.00)) // Go to backdrop
                                .addTemporalMarker(() -> System.out.println("Lowering arm")) // Lower arm
                                .waitSeconds(WAIT_BEFORE_RELEASE)
                                .addTemporalMarker(() -> System.out.println("Released pixel")) // Release pixel
                                .waitSeconds(WAIT_AFTER_RELEASE)
                                .addTemporalMarker(() -> System.out.println("Raising arm")) // Raise arm
                                .waitSeconds(WAIT_FOR_RAISE)
                                .UNSTABLE_addTemporalMarkerOffset(RESET_ARM_OFFSET, () -> System.out.println("Putting arm down")) // Put down arm
                                .lineTo(new Vector2d(backdropPosX, parkIntermediateY)) // Strafe to the side
                                .lineToLinearHeading(new Pose2d(parkX, parkIntermediateY, Math.toRadians(180.00))) // Park
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}