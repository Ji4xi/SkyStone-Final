package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.MichaelBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ParkLeftBridge extends MichaelBot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(22000);
        YLinearInchesGyro(28,GAGE.FORWARD, 0.7, 0);
        sleep(500);
        XLinearInchesGyro(9, Side.LEFT, 0.7, 0);
    }
}
