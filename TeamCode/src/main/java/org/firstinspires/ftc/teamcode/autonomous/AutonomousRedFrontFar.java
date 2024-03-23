package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Red Front Far", group = "SA_FTC")
public class AutonomousRedFrontFar extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.FRONT;
    }

    @Override
    protected Parking parking() { return Parking.FAR; }
}
