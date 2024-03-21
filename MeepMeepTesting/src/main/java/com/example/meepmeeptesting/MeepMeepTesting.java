package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    enum Alliance {
        RED,
        BLUE
    }
    enum InitialPosition {
        FRONT,
        REAR
    }
    enum SpikePosition {
        NONE,
        LEFT,
        RIGHT,
        CENTER
    }

    static Alliance alliance() {
        return Alliance.BLUE;
    }
    static InitialPosition initialPosition() {
        return InitialPosition.FRONT;
    }
    static SpikePosition spikePosition = SpikePosition.RIGHT;

    static final double TILE_SIZE = 23.4;
    static final double STAGE_SIZE = 70.3;
    static final double ROBOT_SIZE = 18;

    // Go to prop
    static final double SPIKE_Y = 32; // The spike's Y position
    static final double SPIKE_SIDE_X = 3; // How much to move to spike's side
    static final double PIXEL_RELEASE_OFFSET = -0.1; // Offset when to release pixel

    // Backdrop
    static final double WAIT_BEFORE_BACKDROP = 1; // Time to wait before going to backdrop
    static final double ARM_SEQUENCE_OFFSET = 4.5; // Offset when to start preparing arm for placing pixel on backdrop
    static final int ARM_SEQUENCE_TARGET = 500; // Arm lift target
    static final double WAIT_BEFORE_RELEASE = 2; // How much time to wait before releasing pixel
    static final double WAIT_AFTER_RELEASE = 0.5; // How much time to wait after releasing pixel
    static final double RESET_ARM_OFFSET = 1; // Offset when to start putting arm down

    static double advanceToZero(double pos, double distance) {
        return pos - Math.signum(pos) * distance;
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
            startingY = 70 - ROBOT_SIZE / 2;
            startingRotation = Math.toRadians(270);
        }
        else if (alliance() == Alliance.RED) {
            startingY = -70 + ROBOT_SIZE / 2;
            startingRotation = Math.toRadians(90);
        }

        Pose2d startingPosition = new Pose2d(startingX, startingY, startingRotation);

        // Go to prop
        double spikePosX = startingPosition.getX();
        double spikePosY = startingPosition.getY();
        double spikeRot = startingPosition.getHeading();

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
        } else {
            spikePosY = startingPosition.getY();
        }

        // Go to backdrop
        double backdropPosX = STAGE_SIZE - TILE_SIZE;
        double backdropPosY = advanceToZero(alliance() == Alliance.BLUE ? STAGE_SIZE : -STAGE_SIZE, TILE_SIZE * 1.5);
        Pose2d backdropPose = new Pose2d(backdropPosX, backdropPosY, Math.toRadians(0.00));

        double spikeCenterX = startingPosition.getX();

        // Park
        double parkIntermediateY = advanceToZero(backdropPosY, TILE_SIZE);
        double parkX = STAGE_SIZE - TILE_SIZE / 2;

        double finalSpikePosX = spikePosX;
        double finalSpikePosY = spikePosY;
        double finalSpikeRot = spikeRot;

        RoadRunnerBotEntity myBot;
        if (initialPosition() == InitialPosition.FRONT) {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
                    .setStartPose(startingPosition)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(startingPosition)
                                    .lineTo(new Vector2d(spikeCenterX, backdropPosY))
                                    .splineTo(new Vector2d(finalSpikePosX, finalSpikePosY), finalSpikeRot)
                                    .waitSeconds(WAIT_BEFORE_BACKDROP)
                                    .lineToLinearHeading(backdropPose)
                                    .waitSeconds(WAIT_BEFORE_RELEASE)
                                    .waitSeconds(WAIT_AFTER_RELEASE)
                                    .lineTo(new Vector2d(backdropPosX, parkIntermediateY))
                                    .lineTo(new Vector2d(parkX, parkIntermediateY))
                                    .build()
                    );
        }
        else {
            myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
                    .setStartPose(startingPosition)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(startingPosition)
                                    .lineTo(new Vector2d(spikeCenterX, backdropPosY))
                                    .splineTo(new Vector2d(finalSpikePosX, finalSpikePosY), finalSpikeRot)
                                    .waitSeconds(WAIT_BEFORE_BACKDROP)
                                    .lineToLinearHeading(new Pose2d(spikeCenterX, backdropPosY, Math.toRadians(0.00)))
                                    .lineToLinearHeading(backdropPose)
                                    .waitSeconds(WAIT_BEFORE_RELEASE)
                                    .waitSeconds(WAIT_AFTER_RELEASE)
                                    .lineTo(new Vector2d(backdropPosX, parkIntermediateY))
                                    .lineTo(new Vector2d(parkX, parkIntermediateY))
                                    .build()
                    );
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}