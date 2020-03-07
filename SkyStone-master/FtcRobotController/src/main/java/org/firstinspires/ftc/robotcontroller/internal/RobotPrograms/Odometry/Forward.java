package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class Forward extends MyOdometryOpmode {
    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        sleep(20000);

        goToPositionSupreme(0, 17.5, maxPwr, 90, 1);
    }
}
