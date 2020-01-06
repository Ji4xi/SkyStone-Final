package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
public class SkyScraperPullBlueMiddle extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        long sleepTime = 700;


        moveForwardInchesGyro(48, 0, sleepTime);
        turnAbsolutePID(90, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime, "turn counterclockwise 90 deg");
        moveForwardInchesGyro(4.25, 90, sleepTime);
        moveFoundationServos(0);
        sleep(1100, "hook on");
        turnAbsolute(145, Direction.COUNTERCLOCKWISE, 1);
        sleep(sleepTime, "turn 55 degrees more counterclockwise");
        moveForwardInchesGyro(42, 145, sleepTime);
        turnAbsolute(180, Direction.COUNTERCLOCKWISE, 1);
        sleep(sleepTime, "turn 35 degrees more counterclockwise");
        moveFoundationServos(1);
        sleep(sleepTime, "unhook claw for foundation");
        moveForwardInches(-5, sleepTime);
        turnAbsolutePID(-125, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward bridge -125");
        moveForwardInchesGyro(36, -125, sleepTime);
        turnAbsolutePID(-90, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward bridge -90");
        moveForwardInches(12, sleepTime);

    }
}
