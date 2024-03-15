package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmUtils {
    // Roller
    final double ROLLER_FLAT = 0.29;
    final double ROLLER_UPSIDEDOWN = 0.97;
    final double ROLLER_ARM_LIMIT = 400;

    // Arm
    final double ARM_EXTEND_SPEED = 0.5;
    final double ARM_LIFT_SPEED = 20;
    final double ARM_LIFT_POWER = 0.4;
    final int ARM_MAX_POSITION = 3200;
    final int ARM_MIN_POSITION = -200;
    final int ARM_EXTEND_LIMIT = 3600;

    // Grip
    final double GRIP_OPEN = 0.5;
    final double GRIP_CLOSED = 0.2;
    final double GRIP_TRIGGER_THRESHOLD = 0.1;

    // Startup Sequence
    final int ARM_LIFT_POSITION = 400;

    // Pixel Pickup Sequence
    final int PICKUP_EXTEND_TARGET = 1700;

    // Pixel Backdrop Sequence
    final int[] BACKDROP_EXTEND_TARGET = { 1800, 2000, 2200 };
    final int[] BACKDROP_ARM_TARGET = { 800, 1000, 1200 };

    // All Sequences
    final double SEQUENCE_ARM_POWER = 0.5;

    // Drone
    final double DRONE_SHOOT = 0.4;

    DcMotor armLift;
    DcMotor armExtend;
    Servo rightGrip;
    Servo leftGrip;
    Servo rollerServo;
    Servo droneServo;

    int currentArmLiftPos = 0;

    int backdropMode = 0;
    boolean prevChangedMode = false;

    boolean startupSequenceActive = false;
    boolean pickupSequenceActive = false;
    boolean backdropSequenceActive = false;
    boolean sequenceActive = false;

    ExtendDirection sequenceDirection = ExtendDirection.UNINITIALIZED;
    boolean sequenceGotToPosition = false;

    public ArmUtils(HardwareMap hardwareMap) {
        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("gripR");
        leftGrip = hardwareMap.servo.get("gripL");
        rollerServo = hardwareMap.servo.get("roll");
        droneServo = hardwareMap.servo.get("droneServo");

        rightGrip.setDirection(Servo.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setTargetPosition(ARM_MIN_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(ARM_LIFT_POWER);
    }

    public void startupSequence() {
        startupSequenceActive = true;

        leftGrip.setPosition(GRIP_CLOSED);
        rightGrip.setPosition(GRIP_CLOSED);
        armLift.setPower(SEQUENCE_ARM_POWER);
        currentArmLiftPos = ARM_LIFT_POSITION;
        armLift.setTargetPosition(currentArmLiftPos);
        armExtend.setPower(-ARM_EXTEND_SPEED);

        if (!armLift.isBusy()) {
            armLift.setPower(ARM_LIFT_POWER);
            armExtend.setPower(0);

            rollerServo.setPosition(ROLLER_FLAT);

            startupSequenceActive = false;
        }
    }

    void pixelPickupSequence() {
        pickupSequenceActive = baseSequence(ARM_MIN_POSITION, PICKUP_EXTEND_TARGET, ROLLER_FLAT, GRIP_OPEN);
    }

    void pixelBackdropSequence() {
        backdropSequenceActive = baseSequence(BACKDROP_ARM_TARGET[backdropMode], BACKDROP_EXTEND_TARGET[backdropMode], ROLLER_FLAT, leftGrip.getPosition());
    }

    boolean baseSequence(int armTarget, int extendTarget, double rollerTarget, double gripTarget) {
        armLift.setPower(SEQUENCE_ARM_POWER);
        currentArmLiftPos = armTarget;
        armLift.setTargetPosition(currentArmLiftPos);
        leftGrip.setPosition(gripTarget);
        rightGrip.setPosition(gripTarget);
        rollerServo.setPosition(rollerTarget);

        if (!sequenceGotToPosition) {
            if (-armExtend.getCurrentPosition() < extendTarget && sequenceDirection != ExtendDirection.BACKWARD) {
                sequenceDirection = ExtendDirection.FORWARD;
                armExtend.setPower(ARM_EXTEND_SPEED);
            }
            else if (-armExtend.getCurrentPosition() > extendTarget && sequenceDirection != ExtendDirection.FORWARD) {
                sequenceDirection = ExtendDirection.BACKWARD;
                armExtend.setPower(-ARM_EXTEND_SPEED);
            }

            sequenceGotToPosition = (sequenceDirection == ExtendDirection.FORWARD && -armExtend.getCurrentPosition() >= extendTarget) || (sequenceDirection == ExtendDirection.BACKWARD && -armExtend.getCurrentPosition() <= extendTarget);
        }
        else {
            armExtend.setPower(0);
        }

        if (!armLift.isBusy() && sequenceGotToPosition) {
            sequenceDirection = ExtendDirection.UNINITIALIZED;
            sequenceGotToPosition = false;

            armExtend.setPower(0);
            armLift.setPower(ARM_LIFT_POWER);

            return false;
        }

        return true;
    }

    public void runSequences(Gamepad gamepad) {
        sequenceActive = startupSequenceActive || backdropSequenceActive || pickupSequenceActive;

        // Backdrop Sequence Mode
        if (!prevChangedMode) {
            if (gamepad.dpad_up) {
                backdropMode++;
                gamepad.rumble(100);
            }
            else if (gamepad.dpad_down) {
                backdropMode--;
                gamepad.rumble(100);
            }
        }

        prevChangedMode = gamepad.dpad_up || gamepad.dpad_down;

        // Start Sequences
        if (!sequenceActive) {
            if (gamepad.a) {
                pixelPickupSequence();
            }
            else if (gamepad.y) {
                pixelBackdropSequence();
            }
        }

        // Run Sequences
        if (startupSequenceActive) {
            startupSequence();
        }
        else if (pickupSequenceActive) {
            pixelPickupSequence();
        }
        else if (backdropSequenceActive) {
            pixelBackdropSequence();
        }
    }

    void stopSequences() {
        startupSequenceActive = false;
        backdropSequenceActive = false;
        pickupSequenceActive = false;
    }

    public void roller(Gamepad gamepad) {
        if (sequenceActive) return;

        if (currentArmLiftPos >= ROLLER_ARM_LIMIT) {
            if (gamepad.x) {
                rollerServo.setPosition(ROLLER_UPSIDEDOWN);
            }

            if (gamepad.b) {
                rollerServo.setPosition(ROLLER_FLAT);
            }
        }
    }

    public void extend(Gamepad gamepad) {
        if (sequenceActive) stopSequences();
        
        if (-armExtend.getCurrentPosition() < ARM_EXTEND_LIMIT || -gamepad.right_stick_y < 0) {
            armExtend.setPower(-gamepad.right_stick_y * ARM_EXTEND_SPEED);
        }
        else armExtend.setPower(0);
    }

    public void lift(Gamepad gamepad) {
        if (sequenceActive) stopSequences();
        
        currentArmLiftPos -= (int)(gamepad.left_stick_y * ARM_LIFT_SPEED);
        if (currentArmLiftPos < ARM_MIN_POSITION && -gamepad.left_stick_y < 0) currentArmLiftPos = ARM_MIN_POSITION;
        if (currentArmLiftPos > ARM_MAX_POSITION && -gamepad.left_stick_y > 0) currentArmLiftPos = ARM_MAX_POSITION;

        armLift.setTargetPosition(currentArmLiftPos);
    }

    public void grip(Gamepad gamepad) {
        if (sequenceActive) return;

        if (gamepad.left_bumper) {
            leftGrip.setPosition(GRIP_OPEN);
        }

        if (gamepad.right_bumper) {
            rightGrip.setPosition(GRIP_OPEN);
        }

        if (gamepad.left_trigger > GRIP_TRIGGER_THRESHOLD) {
            leftGrip.setPosition(GRIP_CLOSED);
        }

        if (gamepad.right_trigger > GRIP_TRIGGER_THRESHOLD) {
            rightGrip.setPosition(GRIP_CLOSED);
        }
    }

    public void drone(Gamepad gamepad) {
        if (gamepad.guide) {
            droneServo.setPosition(DRONE_SHOOT);
            sleep(100);
        }
    }
}

enum ExtendDirection {
    UNINITIALIZED,
    FORWARD,
    BACKWARD
}
