package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Blue Front Far", group = "SA_FTC")
public class AutonomousBlueFrontFar extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.FRONT;
    }

    @Override
    protected Parking parking() { return Parking.FAR; }
}
