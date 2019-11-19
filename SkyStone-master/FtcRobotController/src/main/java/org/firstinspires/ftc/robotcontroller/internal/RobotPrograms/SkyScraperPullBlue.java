package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SkyScraperPullBlue extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        moveForwardInches(39.5);
        sleep(3000, "go forward 39.5 inches");
        turnAbsolute(90, Direction.COUNTERCLOCKWISE, 0.6);
        sleep(3000, "turn counterclockwise 90 deg");
        moveForwardInches(2);
        sleep(3000, "go forward 2 inches");
        rs.setPosition(0);
        ls.setPosition(0);
        sleep(3000, "hook on");
        turnAbsolute(-45, Direction.COUNTERCLOCKWISE, 0.6);
        sleep(3000, "turn 135 deg clockwise");
        moveForwardInches(6);
        sleep(3000, "go forward 6 inches");
        moveFoundationServos(1);
        sleep(3000, "unlower claw for foundation");
    }
}
