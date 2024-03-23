package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Blue Front Near", group = "SA_FTC")
public class AutonomousBlueFrontNear extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.FRONT;
    }

    @Override
    protected Parking parking() { return Parking.NEAR; }
}
