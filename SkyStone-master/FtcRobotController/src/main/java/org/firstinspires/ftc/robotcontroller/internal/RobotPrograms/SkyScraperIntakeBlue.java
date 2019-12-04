package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class SkyScraperIntakeBlue extends SkyScraper{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        long sleepTime = 1000;
        //state 1
        moveForwardInches(25, sleepTime);
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
        rm.setPower(0.2);
        lm.setPower(0.2);
        if (FoundSkyStoneAngle()) {
            stopMotor();
            turnAbsolutePID(-180, Direction.CLOCKWISE, 0.3);
//        moveForwardInches();
        }

    }
}
