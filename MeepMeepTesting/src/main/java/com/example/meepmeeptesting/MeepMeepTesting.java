package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.74330378369762, 28.78180821614297, 3.211860967940206, 3.211860967940206, 21.77)
                .setStartPose(new Pose2d(-36.33, 62.75, Math.toRadians(270.00)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.33, 62.75, Math.toRadians(270.00)))
                                .splineTo(new Vector2d(-22.36, 0.85), Math.toRadians(0.47))
                                .splineTo(new Vector2d(61.43, 0.28), Math.toRadians(0.00))
                                .lineToConstantHeading(new Vector2d(61.43, 14.44))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}