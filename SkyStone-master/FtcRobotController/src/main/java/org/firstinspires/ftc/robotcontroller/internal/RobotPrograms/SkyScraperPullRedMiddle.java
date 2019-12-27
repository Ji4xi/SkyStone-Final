package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
public class SkyScraperPullRedMiddle extends SkyScraperPullBlueMiddle {

    @Override
    public void runOpMode() throws InterruptedException {
        mode = false;
        super.runOpMode();
    }
}
