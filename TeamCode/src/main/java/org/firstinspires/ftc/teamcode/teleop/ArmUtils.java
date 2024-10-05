package org.firstinspires.ftc.teamcode.teleop;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmUtils {
    // Roller
    public static final double ROLLER_FLAT = 0.29;
    public static final double ROLLER_UPSIDEDOWN = 0.97;
    public static final double ROLLER_ARM_LIMIT = 200;

    // Arm
    public static final double ARM_EXTEND_SPEED = 0.5;
    public static final double ARM_LIFT_SPEED = 40;
    public static final double ARM_LIFT_POWER = 0.6;
    public static final int ARM_MAX_POSITION = 3200;
    public static final int ARM_MIN_POSITION = -200;
    public static final int ARM_EXTEND_MAX_LIMIT = 3600;
    public static final int ARM_EXTEND_MIN_LIMIT = 500;

    // Grip
    public static final double GRIP_OPEN = 0.5;
    public static final double GRIP_CLOSED = 0.2;
    public static final double GRIP_TRIGGER_THRESHOLD = 0.1;

    // Startup Sequence
    final int ARM_LIFT_POSITION = 400;

    // Pixel Pickup Sequence
    final int PICKUP_EXTEND_TARGET = 1700;

    // Pixel Backdrop Sequence
    public static final int[] BACKDROP_EXTEND_TARGET = { 1300, 1700, 2100 };
    public static final int BACKDROP_ARM_TARGET = 1000;

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
    DigitalChannel leftSwitch;
    DigitalChannel rightSwitch;

    int currentArmLiftPos = 0;

    int backdropMode = 0;
    boolean prevChangedMode = false;

    boolean startupSequenceActive = false;
    boolean pickupSequenceActive = false;
    boolean backdropSequenceActive = false;
    boolean sequenceActive = false;

    ExtendDirection sequenceDirection = ExtendDirection.UNINITIALIZED;
    boolean sequenceGotToPosition = false;
    boolean sequenceFirstTime = true;

    public ArmUtils(HardwareMap hardwareMap) {
        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("gripR");
        leftGrip = hardwareMap.servo.get("gripL");
        rollerServo = hardwareMap.servo.get("roll");
        droneServo = hardwareMap.servo.get("droneServo");
        leftSwitch = hardwareMap.digitalChannel.get("leftSwitch");
        rightSwitch = hardwareMap.digitalChannel.get("rightSwitch");

        rightGrip.setDirection(Servo.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setTargetPosition(0);
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
        double leftGripTarget = !leftSwitch.getState() ? GRIP_CLOSED : GRIP_OPEN;
        double rightGripTarget = !rightSwitch.getState() ? GRIP_CLOSED : GRIP_OPEN;

        pickupSequenceActive = baseSequence(ARM_MIN_POSITION, PICKUP_EXTEND_TARGET, ROLLER_FLAT, leftGripTarget, rightGripTarget);
    }

    void pixelBackdropSequence() {
        backdropSequenceActive = baseSequence(BACKDROP_ARM_TARGET, BACKDROP_EXTEND_TARGET[backdropMode], ROLLER_FLAT, leftGrip.getPosition(), rightGrip.getPosition());
    }

    boolean baseSequence(int armTarget, int extendTarget, double rollerTarget, double leftGripTarget, double rightGripTarget) {
        if (sequenceFirstTime) {
            sequenceFirstTime = false;

            armLift.setPower(SEQUENCE_ARM_POWER);
            currentArmLiftPos = armTarget;
            armLift.setTargetPosition(currentArmLiftPos);
            leftGrip.setPosition(leftGripTarget);
            rightGrip.setPosition(rightGripTarget);
            rollerServo.setPosition(rollerTarget);
        }

        if (!sequenceGotToPosition) {
            if (-armExtend.getCurrentPosition() < extendTarget && sequenceDirection != ExtendDirection.BACKWARD) {
                sequenceDirection = ExtendDirection.FORWARD;
                armExtend.setPower(ARM_EXTEND_SPEED);
            }
            else if (-armExtend.getCurrentPosition() > extendTarget && sequenceDirection != ExtendDirection.FORWARD) {
                sequenceDirection = ExtendDirection.BACKWARD;
                armExtend.setPower(-ARM_EXTEND_SPEED);
            }
            else {
                sequenceGotToPosition = true;
            }

            sequenceGotToPosition = (sequenceDirection == ExtendDirection.FORWARD && -armExtend.getCurrentPosition() >= extendTarget) || (sequenceDirection == ExtendDirection.BACKWARD && -armExtend.getCurrentPosition() <= extendTarget);
        }
        else {
            armExtend.setPower(0);
        }

        if (!armLift.isBusy() && sequenceGotToPosition) {
            sequenceDirection = ExtendDirection.UNINITIALIZED;
            sequenceGotToPosition = false;
            sequenceFirstTime = true;

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
                if (backdropMode < BACKDROP_EXTEND_TARGET.length - 1) backdropMode++;

                gamepad.rumble(150);
                stopSequences();
                pixelBackdropSequence();
            }
            else if (gamepad.dpad_down) {
                if (backdropMode > 0) backdropMode--;

                gamepad.rumble(150);
                stopSequences();
                pixelBackdropSequence();
            }
        }

        prevChangedMode = gamepad.dpad_up || gamepad.dpad_down;

        // Start Sequences
        if (!sequenceActive) {
            if (gamepad.a) {
                stopSequences();
                pixelPickupSequence();
            }
            else if (gamepad.y) {
                stopSequences();
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
        if (sequenceActive && gamepad.right_stick_y != 0) stopSequences();

        if ((-armExtend.getCurrentPosition() < ARM_EXTEND_MAX_LIMIT || -gamepad.right_stick_y < 0) && (-armExtend.getCurrentPosition() > ARM_EXTEND_MIN_LIMIT || -gamepad.right_stick_y > 0)) {
            armExtend.setPower(-gamepad.right_stick_y * ARM_EXTEND_SPEED);
        }
        else armExtend.setPower(0);
    }

    public void lift(Gamepad gamepad) {
        if (sequenceActive && gamepad.left_stick_y != 0) stopSequences();

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

    public enum ExtendDirection {
        UNINITIALIZED,
        FORWARD,
        BACKWARD
    }
}