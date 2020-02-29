package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class Right extends MyOdometryOpmode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        sleep(29000-how many ever milliseconds it takes for the program);
        //park middle
        goToPosition(0, -46+17.5+1, 0.3, 90);
        goToPosition(17.5, -46+17.5+1, 0.3, 90);
    }
}
