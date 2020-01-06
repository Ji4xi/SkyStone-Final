package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper.SkyScraper;

//@Disabled
@Autonomous
public class Auto1 extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        moveForward(3);
    }
}
