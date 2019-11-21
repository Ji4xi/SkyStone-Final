package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.RangerAuto;

@Disabled
@Autonomous
public class LeftTurnWall extends RangerAuto {

    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        moveForwardInches(4);
        turnAbsolute(90, Direction.COUNTERCLOCKWISE, TURN_SPEED);
        moveForwardInches(34.5);
    }
}
