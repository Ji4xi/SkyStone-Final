package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class ParkOutsideLeft extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        long sleepTime = 700;

        sleep(15000);
        moveForwardInchesGyro(24, 0, sleepTime);
        turnAbsolutePID(90, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime);
        moveForwardInchesGyro(12, 90, sleepTime);
    }
    
}
