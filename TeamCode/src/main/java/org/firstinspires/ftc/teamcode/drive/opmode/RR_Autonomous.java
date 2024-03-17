package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "TestRoadrunner", group = "SA_FTC")
public class RR_Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(new Pose2d(0,0, 0));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 30), 0)
                .build();

        //drive.followTrajectory(trajectory1);

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(-36.33, 62.75, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-22.36, 0.85), Math.toRadians(0.47))
                .splineTo(new Vector2d(61.43, 0.28), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(61.43, 14.44))
                .build();
        drive.setPoseEstimate(trajectory2.start());

        drive.followTrajectorySequence(trajectory2);
        //PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
