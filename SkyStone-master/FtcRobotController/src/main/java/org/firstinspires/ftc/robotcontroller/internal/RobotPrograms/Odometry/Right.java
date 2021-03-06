package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

@Autonomous

public class Right extends NewStoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(20000);
        //park middle
        goToPositionSupreme(17.5, 46-17.5-1, maxPwr, 90, 1);
    }
}
