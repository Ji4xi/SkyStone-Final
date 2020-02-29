package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class FoundationBlueMiddle extends FoundationBlueSide{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        sleep(29000-how many ever milliseconds it takes for the program);
        moveFoundation();
        //park middle
        goToPosition(22.75-17.5/2+22.75, 0, 0.3, 90);
        goToPosition(22.75-17.5/2+22.75, -46+17.5+1, 0.3, 90);
        goToPosition(22.75-17.5/2 + 22.75 + 17.5, -46+17.5+1, 0.3, 90);


    }
}
