package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class FoundationBlueMiddle extends FoundationBlueSide{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        sleep(29000-how many ever milliseconds it takes for the program);
        moveFoundation();
        //park middle
        goToPositionSupreme(-22.75+17.5-22.75+17.5/2, 0, maxPwr, 90, 1);
        goToPositionSupreme(-22.75+17.5-22.75+17.5/2, -46+17.5+1, maxPwr, 90, 1);
        goToPositionSupreme(-22.75+17.5-22.75, -46+17.5+1, maxPwr, 90, 1);


    }
}
