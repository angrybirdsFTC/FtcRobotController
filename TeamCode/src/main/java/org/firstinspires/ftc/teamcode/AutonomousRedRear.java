package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Red Rear", group = "SA_FTC")
public class AutonomousRedRear extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.REAR;
    }
}
