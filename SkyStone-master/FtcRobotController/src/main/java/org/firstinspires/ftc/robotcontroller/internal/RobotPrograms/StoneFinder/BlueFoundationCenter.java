package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class BlueFoundationCenter extends BlueFoundation{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        YLinearInchesGyro(17, GAGE.REVERSE, 0.30, 0);
    }
}
