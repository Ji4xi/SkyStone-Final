package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SkyScraperIntakeRed extends SkyScraperIntakeBlue {

    @Override
    public void runOpMode() throws InterruptedException {
        mode = false;
        super.runOpMode();
    }
}
