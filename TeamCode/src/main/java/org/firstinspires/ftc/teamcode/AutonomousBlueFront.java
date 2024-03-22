package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Blue Front", group = "SA_FTC")
public class AutonomousBlueFront extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.FRONT;
    }
}
