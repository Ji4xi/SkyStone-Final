package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class StoneFinderIntakeRed extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        moveInchesPID(10, skystoneAngle, 1000);
        moveClaw(1);
        moveInchesPID(30, 90, 1000);
        moveClaw(0);
        //second brick
        moveInchesPID(54, 180, 1000);
        moveClaw(1);
        moveInchesPID(84, 90, 1000);
        moveInchesPID(12, 180, 1000);

    }
}
