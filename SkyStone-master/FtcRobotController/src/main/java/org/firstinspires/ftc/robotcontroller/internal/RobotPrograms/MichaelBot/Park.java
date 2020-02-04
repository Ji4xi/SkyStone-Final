package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.MichaelBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Park extends MichaelBot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(27000);
        XLinearInchesGyro(19, Side.LEFT, 0.7, 0);
    }
}
