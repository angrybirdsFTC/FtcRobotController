package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DrivingAssist {
    DigitalChannel leftSwitch;
    DigitalChannel rightSwitch;
    DigitalChannel led1;
    DigitalChannel led2;

    boolean prevLeftSwitchState = false;
    boolean prevRightSwitchState = false;

    boolean didEndgameRumble = false;

    public DrivingAssist(HardwareMap hardwareMap) {
        leftSwitch = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");
        led1 = hardwareMap.get(DigitalChannel.class, "led1");
        led2 = hardwareMap.get(DigitalChannel.class, "led2");
        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void gripLed(Gamepad gamepad1, Gamepad gamepad2) {
        led1.setState(!leftSwitch.getState());
        led2.setState(!rightSwitch.getState());

        // Check if previous state is false and current state is true
        if (prevLeftSwitchState && !leftSwitch.getState()) {
            gamepad1.rumbleBlips(1);
            gamepad2.rumbleBlips(1);
        }
        if (prevRightSwitchState && !rightSwitch.getState()) {
            gamepad1.rumbleBlips(2);
            gamepad2.rumbleBlips(2);
        }

        prevLeftSwitchState = leftSwitch.getState();
        prevRightSwitchState = rightSwitch.getState();
    }

    public void endgameCountdown(Gamepad gamepad1, Gamepad gamepad2, ElapsedTime elapsedTime) {
        if (elapsedTime.seconds() >= 90 && !didEndgameRumble) {
            didEndgameRumble = true;

            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }
    }
}
