package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraperIntakeBlue;

@Disabled
@Autonomous
public class ParkInsider extends SkyScraper {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(25000);
        moveForwardInchesGyro(10, 0, 3);
    }
}
