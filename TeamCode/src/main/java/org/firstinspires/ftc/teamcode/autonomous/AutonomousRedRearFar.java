package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Red Rear Far", group = "SA_FTC")
public class AutonomousRedRearFar extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.REAR;
    }

    @Override
    protected Parking parking() { return Parking.FAR; }
}
