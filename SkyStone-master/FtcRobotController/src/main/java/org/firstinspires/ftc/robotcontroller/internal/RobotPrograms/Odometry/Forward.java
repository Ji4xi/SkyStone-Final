package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class Forward extends MyOdometryOpmode {
    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
//        sleep(29000-how many ever milliseconds it takes for the program);

        goToPositionSupreme(0, 17.5, maxPwr, 90, 1);
    }
}
