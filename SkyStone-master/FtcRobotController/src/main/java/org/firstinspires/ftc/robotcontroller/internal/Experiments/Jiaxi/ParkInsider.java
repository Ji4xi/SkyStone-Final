package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraperIntakeBlue;

@Autonomous
public class ParkInsider extends SkyScraper {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(25000);
        moveForwardInchesGyro(10, 0, 3);
    }
}
