package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
public class LeftTurnWall extends CalvinRangerAuto {

    static final double DRIVE_PWR = 0.8;
    static final double TURN_PWR = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        moveForwardInches(4);
        turnAbsolute(90, turnDirection.COUNTERCLOCKWISE, TURN_PWR);
        moveForwardInches(34.5);
    }
}
