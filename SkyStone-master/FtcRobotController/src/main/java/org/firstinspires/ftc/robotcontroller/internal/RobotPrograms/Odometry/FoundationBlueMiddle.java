package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FoundationBlueMiddle extends FoundationBlueSide{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(20000);
        moveFoundation();
        //park middle
        goToPositionSupreme(-22.75+17.5-22.75+17.5/2, 0, maxPwr, 90, 1);
        goToPositionSupreme(-22.75+17.5-22.75+17.5/2, -46+17.5+1, maxPwr, 90, 1);
        goToPositionSupreme(-22.75+17.5-22.75, -46+17.5+1, maxPwr, 90, 1);


    }
}
