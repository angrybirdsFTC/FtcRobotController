package org.firstinspires.ftc.teamcode;

import androidx.appcompat.widget.VectorEnabledTintResources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

public abstract class MainAutonomous extends LinearOpMode {
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

    SampleMecanumDrive drive;

    void runAutonomous(Pose2d startingPosition) {
        // TODO: Detect prop

        // Center
        TrajectorySequence goToSpikeCenter = drive.trajectorySequenceBuilder(startingPosition)
                .lineToConstantHeading(new Vector2d(startingPosition.getX(), 32.84)) // y - 32.84 
                .build();
        drive.setPoseEstimate(goToSpikeCenter.start());
    }

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        double startingX;
        double startingY;
        double startingRotation;

        if (initialPosition() == InitialPosition.FRONT) {
            startingX = 11.44;
        }
        else {
            startingX = -36.3;
        }

        if (alliance() == Alliance.RED) {
            startingY = -63.32;
            startingRotation = Math.toRadians(90);
        }
        else {
            startingY = 63.32;
            startingRotation = Math.toRadians(270);
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