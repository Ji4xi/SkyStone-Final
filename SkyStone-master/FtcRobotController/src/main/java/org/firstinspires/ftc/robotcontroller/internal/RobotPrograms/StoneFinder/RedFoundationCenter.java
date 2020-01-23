package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedFoundationCenter extends RedFoundation {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        YLinearInchesGyro(17, GAGE.REVERSE, 0.30, 0);
    }
}
