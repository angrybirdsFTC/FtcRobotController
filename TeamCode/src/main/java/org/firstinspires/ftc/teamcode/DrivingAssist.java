package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class DrivingAssist {
    final int MAX_TICKS = 100;

    DigitalChannel leftSwitch;
    DigitalChannel rightSwitch;
    DigitalChannel led1;
    DigitalChannel led2;

    Controller controller;


    int ticksRemaining = 0;
    //double tempSpeed = 0;
    boolean prevLeftSwitchState = false;
    boolean prevRightSwitchState = false;

    public DrivingAssist(Controller controller, HardwareMap hardwareMap) {
        this.controller = controller;


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
        else if (prevRightSwitchState && !rightSwitch.getState()) {
            gamepad1.rumbleBlips(2);
            gamepad2.rumbleBlips(2);
        }

        prevLeftSwitchState = leftSwitch.getState();
        prevRightSwitchState = rightSwitch.getState();
    }

    void rumbleSpeed(Gamepad gamepad1, Gamepad gamepad2, double speed) {
        if (speed > 1 || speed < 0) return;

        if (ticksRemaining == 0) {
            gamepad1.rumble(100);
            gamepad2.rumble(100);
            ticksRemaining = MAX_TICKS - (int)(speed * MAX_TICKS);
        }
        else {
            ticksRemaining--;
        }
    }
}
