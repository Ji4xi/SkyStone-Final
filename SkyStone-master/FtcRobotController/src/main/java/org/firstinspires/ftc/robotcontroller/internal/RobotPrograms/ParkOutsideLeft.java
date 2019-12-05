package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper;

@Autonomous
public class ParkOutsideLeft extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(20000);
        moveForwardInchesGyro(36, 0, 2);
        turnAbsolutePID(90, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(2000);
        moveForwardInchesGyro(7, 2);
    }
    
}
