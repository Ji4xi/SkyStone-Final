package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedFoundation extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        foundation(Mode.OPEN,2000);
        moveInchesPID(28, 0, 2000);
        moveInchesPID(30.25, 90, 2000);
        foundation(Mode.CLOSE, 2000);
        moveInchesPID(30.25, 270, 2000);
        foundation(Mode.OPEN,2000);
        moveInchesPID(63.25, 180, 2000);

    }
}
