package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

public class StoneFinderTest extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        moveInches(3 * WHEEL_PERIMETER_INCHES, 0, 2000);
    }
}
