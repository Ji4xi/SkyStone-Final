package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SkyScraperPullRedSide extends SkyScraperPullBlueSide {

    @Override
    public void runOpMode() throws InterruptedException {
        mode = false;
        super.runOpMode();
    }

}