package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Right extends MyOdometryOpmode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(20000);
        //park middle
        goToPositionSupreme(17.5, 46-17.5-1, maxPwr, 90, 1);
    }
}
