package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class Right extends MyOdometryOpmode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        sleep(29000-how many ever milliseconds it takes for the program);
        //park middle
        goToPositionSupreme(0, 46-17.5-1, maxPwr, 90, 1);
        goToPositionSupreme(17.5, 46-17.5-1, maxPwr, 90, 1);
    }
}
