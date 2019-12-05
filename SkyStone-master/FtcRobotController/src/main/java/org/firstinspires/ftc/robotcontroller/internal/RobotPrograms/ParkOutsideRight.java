package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkOutsideRight extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(20000);
        moveForwardInchesGyro(12, 0, 2000);
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
        sleep(2000);
        moveForwardInchesGyro(7, 0, 2000);
    }
    
}
