package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Blue Rear Near", group = "SA_FTC")
public class AutonomousBlueRearNear extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.REAR;
    }

    @Override
    protected Parking parking() { return Parking.NEAR; }
}
