package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

public class BlueFoundation extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        foundation(StoneFinder.Mode.OPEN,2000);
        moveInchesPID(28, 180, 2000);
        moveInchesPID(30.25, 90, 2000);
        foundation(StoneFinder.Mode.CLOSE, 2000);
        moveInchesPID(30.25, 270, 2000);
        foundation(StoneFinder.Mode.OPEN,2000);
        moveInchesPID(63.25, 0, 2000);

    }
}