package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "led test", group = "SA_FTC")
//@Autonomous
public class TestLed extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor ledMotor;
    private DcMotor ledMotor1;

    @Override
    public void runOpMode() {
        ledMotor = hardwareMap.get(DcMotor.class, "ledmotor");
        ledMotor1 = hardwareMap.get(DcMotor.class, "ledmotor2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // System.out.print(imu.getAngularOrientation());
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            if (gamepad1.left_bumper) {
                ledMotor.setPower(0.5);
            } else {
                ledMotor.setPower(0);

            }
            if (gamepad1.right_bumper) {
                ledMotor1.setPower(0.5);
            } else {
                ledMotor1.setPower(0);

            }

        }
    }
}
