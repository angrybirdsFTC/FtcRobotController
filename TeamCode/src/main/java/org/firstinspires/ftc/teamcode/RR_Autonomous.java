package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

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

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(-36.33, 62.37, Math.toRadians(-90)))
                .splineTo(new Vector2d(-20.29, 11.23), Math.toRadians(-2.96))
                .splineTo(new Vector2d(60.87, 12.36), Math.toRadians(0.00))
                .build();
        drive.setPoseEstimate(trajectory2.start());

        drive.followTrajectorySequence(trajectory2);
        //PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
