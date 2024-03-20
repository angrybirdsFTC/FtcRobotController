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
        return Alliance.RED;
    }
    static InitialPosition initialPosition() {
        return InitialPosition.FRONT;
    }
    static SpikePosition spikePosition = SpikePosition.LEFT;

    static final double TILE_SIZE = 23.4;
    static final double ROBOT_SIZE = 18;

    // Go to prop
    static final double SPIKE_Y = 32;
    static final double SPIKE_SIDE_X = 3;

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
        double posX = startingPosition.getX();
        double posY = startingPosition.getY();
        double rot = startingPosition.getHeading();

        if (spikePosition == SpikePosition.CENTER) {
            posY = advanceToZero(posY, SPIKE_Y);
        }
        else if (spikePosition == SpikePosition.LEFT) {
            posY = advanceToZero(posY, SPIKE_Y);

            if (alliance() == Alliance.BLUE) {
                posX += SPIKE_SIDE_X;
                rot = Math.toRadians(0);
            }
            else if (alliance() == Alliance.RED) {
                posX -= SPIKE_SIDE_X;
                rot = Math.toRadians(180);
            }
        }
        else if (spikePosition == SpikePosition.RIGHT) {
            posY = advanceToZero(posY, SPIKE_Y);

            if (alliance() == Alliance.BLUE) {
                posX -= SPIKE_SIDE_X;
                rot = Math.toRadians(180);
            }
            else if (alliance() == Alliance.RED) {
                posX += SPIKE_SIDE_X;
                rot = Math.toRadians(0);
            }
        }

        double finalPosX = posX;
        double finalPosY = posY;
        double finalRot = rot;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
                .setStartPose(startingPosition)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPosition)
                                .splineTo(new Vector2d(finalPosX, finalPosY), finalRot)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}