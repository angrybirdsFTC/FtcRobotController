package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    static SpikePosition spikePosition = SpikePosition.RIGHT;

    static final double TILE_SIZE = 23.4;
    static final double STAGE_SIZE = 70.3;
    static final double ROBOT_SIZE = 17;

    // Go to prop
    static final double SPIKE_Y = 28; // Y distance to spike
    static final double SPIKE_CENTER_Y = 29; // Y distance to spike center
    static final double SPIKE_SIDE_X = 0.1; // How much to move to spike's side
    static final double PIXEL_RELEASE_OFFSET = -0.5; // Offset when to release pixel
    static final double BACK_UP = 2; // Distance to go back after placing pixel

    // Backdrop
    static final double FRONT_DOWN = TILE_SIZE; // Y position to move to after placing pixel on spike in front
    static final double BACKDROP_POS_X = STAGE_SIZE - TILE_SIZE * 0.5 - ROBOT_SIZE / 1.8;
    static final double BACKDROP_CENTER_POS_Y = TILE_SIZE * 1.2; // The backdrop's Y position from the edge of the stage
    static final double BACKDROP_LEFT_OFFSET = TILE_SIZE * 0.3; // How much to move from the center of the backdrop to the left of the backdrop
    static final double BACKDROP_RIGHT_OFFSET = TILE_SIZE * 0.5; // How much to move from the center of the backdrop to the right of the backdrop
    static final double WAIT_BEFORE_BACKDROP = 0.5; // Time to wait before going to backdrop
    static final double EXTEND_OFFSET = 1.5; // How much time to wait after starting sequence before extending arm
    static final int BACKDROP_ARM_TARGET = 1700; // Arm lift target for raising arm
    static final int BACKDROP_EXTEND_TARGET = 1200; // Arm extend target for raising arm
    static final int ARM_SEQUENCE_TARGET = 200; // Arm lift target for lowering arm
    static final double WAIT_BEFORE_RELEASE = 2; // How much time to wait before releasing pixel
    static final double WAIT_AFTER_RELEASE = 0.5; // How much time to wait after releasing pixel
    static final double WAIT_FOR_RAISE = 1; // How much time to wait for the arm to raise
    static final double RESET_ARM_OFFSET = 0.5; // Offset when to start putting arm down

    // Parking
    static final double PARKING_INTERMEDIATE_X = BACKDROP_POS_X - 10;
    static final double NEAR_PARKING = TILE_SIZE * 0.12;
    static final double FAR_PARKING = TILE_SIZE * 2.5;

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

//        // Calculate starting position
//        double startingX = 0;
//        double startingY = 0;
//        double startingRotation = 0;
//
//        if (initialPosition() == InitialPosition.FRONT) {
//            startingX = TILE_SIZE / 2;
//        }
//        else if (initialPosition() == InitialPosition.REAR) {
//            startingX = -(TILE_SIZE * 1.53);
//        }
//
//        if (alliance() == Alliance.BLUE) {
//            startingY = STAGE_SIZE - ROBOT_SIZE / 2;
//            startingRotation = Math.toRadians(270);
//        }
//        else if (alliance() == Alliance.RED) {
//            startingY = -STAGE_SIZE + ROBOT_SIZE / 2;
//            startingRotation = Math.toRadians(90);
//        }
//
//        Pose2d startPose = new Pose2d(startingX, startingY, startingRotation);
//
//        // Go to prop
//        double spikePosX = startPose.getX();
//        double spikePosY = startPose.getY();
//        double spikeRot = startPose.getHeading();
//
//        if (spikePosition == SpikePosition.CENTER) {
//            spikePosY = advanceToZero(spikePosY, SPIKE_CENTER_Y);
//        }
//        else if (spikePosition == SpikePosition.LEFT) {
//            spikePosY = advanceToZero(spikePosY, SPIKE_Y);
//
//            if (alliance() == Alliance.BLUE) {
//                spikePosX += SPIKE_SIDE_X;
//                spikeRot = Math.toRadians(0);
//            }
//            else if (alliance() == Alliance.RED) {
//                spikePosX -= SPIKE_SIDE_X;
//                spikeRot = Math.toRadians(180);
//            }
//        }
//        else if (spikePosition == SpikePosition.RIGHT) {
//            spikePosY = advanceToZero(spikePosY, SPIKE_Y);
//
//            if (alliance() == Alliance.BLUE) {
//                spikePosX -= SPIKE_SIDE_X;
//                spikeRot = Math.toRadians(180);
//            }
//            else if (alliance() == Alliance.RED) {
//                spikePosX += SPIKE_SIDE_X;
//                spikeRot = Math.toRadians(0);
//            }
//        }
//
//        // Go to backdrop
//        double backdropPosY = advanceToZero(getStageEdge(), BACKDROP_CENTER_POS_Y);;
//        if (spikePosition == SpikePosition.LEFT) {
//            backdropPosY += BACKDROP_LEFT_OFFSET;
//        }
//        else if (spikePosition == SpikePosition.RIGHT) {
//            backdropPosY -= BACKDROP_RIGHT_OFFSET;
//        }
//
//        Vector2d backdropPose = new Vector2d(BACKDROP_POS_X, backdropPosY);
//
//        double spikeCenterX = startPose.getX();
//        double spikeCenterY = advanceToZero(getStageEdge(), BACKDROP_CENTER_POS_Y);
//
//        double rearIntermediateY = advanceToZero(getStageEdge(), TILE_SIZE * 2.7);
//
//        // Park
//        double parkIntermediateY = advanceToZero(getStageEdge(), parking() == Parking.NEAR ? NEAR_PARKING : FAR_PARKING);
//        double parkX = STAGE_SIZE - TILE_SIZE / 2;
//
//        double beforeBackdropY = initialPosition() == InitialPosition.FRONT ? advanceToZero(getStageEdge(), FRONT_DOWN) : rearIntermediateY;
//        double beforeBackdropX1;
//        if (spikePosition == SpikePosition.CENTER) {
//            beforeBackdropX1 = initialPosition() == InitialPosition.FRONT ? spikeCenterX + 0.1 : spikeCenterX - TILE_SIZE * 0.6; // + 0.1 for no EmptyPathSegmentException
//        }
//        else {
//            beforeBackdropX1 = spikeCenterX + 0.1;
//        }
//        double beforeBackdropX2 = initialPosition() == InitialPosition.FRONT ? spikeCenterX + 0.1 : 0; // + 0.1 for no EmptyPathSegmentException
//
//        double finalSpikePosX = spikePosX;
//        double finalSpikePosY = spikePosY;
//        double finalSpikeRot = spikeRot;
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
//                .setStartPose(startPose)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPose)
//                                .lineTo(new Vector2d(spikeCenterX, spikeCenterY)) // Go to spike center
//                                .lineTo(new Vector2d(finalSpikePosX, finalSpikePosY)) // Go to specific position on spike
//                                .lineToLinearHeading(new Pose2d(finalSpikePosX + 0.01, finalSpikePosY + 0.01, finalSpikeRot)) // Rotate to spike
//                                .back(BACK_UP) // Move back
//                                .waitSeconds(WAIT_BEFORE_BACKDROP)
//                                .lineTo(new Vector2d(beforeBackdropX1, finalSpikePosY)) // Nothing (front) Go towards back (rear)
//                                .lineTo(new Vector2d(beforeBackdropX1, beforeBackdropY)) // Go up (front) Go forward (rear)
//                                .lineToLinearHeading(new Pose2d(beforeBackdropX1 + 0.1, beforeBackdropY + 0.1, Math.toRadians(0.00))) // Rotate to 0
//                                .lineTo(new Vector2d(beforeBackdropX2, beforeBackdropY)) // Go to center (rear)
//                                .splineToConstantHeading(backdropPose, Math.toRadians(0.00)) // Go to backdrop
//                                .waitSeconds(WAIT_BEFORE_RELEASE)
//                                .waitSeconds(WAIT_AFTER_RELEASE)
//                                .lineTo(new Vector2d(PARKING_INTERMEDIATE_X, backdropPose.getY())) // Go back
//                                .waitSeconds(WAIT_FOR_RAISE)
//                                .lineTo(new Vector2d(PARKING_INTERMEDIATE_X, parkIntermediateY)) // Strafe to the side
//                                .lineToLinearHeading(new Pose2d(PARKING_INTERMEDIATE_X + 0.1, parkIntermediateY + 0.1, Math.toRadians(180.00))) // Rotate to 180
//                                .lineTo(new Vector2d(parkX, parkIntermediateY)) // Park
//                                .build()
//                );

        double yaw = -30;
        double tagX = 59.17;
        double tagY = 29.72;
        double posX = tagX - 30;
        double posY = tagY + -20;

        if (yaw > 0) {
            yaw = 360 - yaw;
        }
        else {
            yaw = -yaw;
        }

        double finalYaw = yaw;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
                .setStartPose(new Pose2d(posX, posY, Math.toRadians(yaw)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(posX, posY, Math.toRadians(finalYaw)))
                                .splineTo(new Vector2d(tagX - 10, tagY), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}