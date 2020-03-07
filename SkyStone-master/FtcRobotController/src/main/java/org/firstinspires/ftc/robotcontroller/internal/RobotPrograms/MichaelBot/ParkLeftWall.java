package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.MichaelBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class ParkLeftWall extends MichaelBot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(27000);
        XLinearInchesGyro(9, Side.LEFT, 0.7, 0);
    }
}
