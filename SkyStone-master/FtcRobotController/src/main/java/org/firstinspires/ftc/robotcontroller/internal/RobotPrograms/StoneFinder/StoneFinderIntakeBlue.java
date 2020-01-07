package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper.SkyScraper;

public class StoneFinderIntakeBlue extends StoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        runToPosition(10, skystoneAngle);
        //turn to 90 here
        moveClaw(42);
        runToPosition(-30, -90);


    }
}
