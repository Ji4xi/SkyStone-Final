package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

@Autonomous

public class Forward extends NewStoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        sleep(20000);

        goToPositionSupreme(0, 17.5, maxPwr, 90, 1);
    }
}
