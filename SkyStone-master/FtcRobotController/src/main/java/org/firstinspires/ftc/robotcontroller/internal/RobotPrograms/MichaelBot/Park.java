package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.MichaelBot;

public class Park extends MichaelBot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(28500);
        XLinearInchesGyro(19, Side.LEFT, 0.7, 0);
    }
}
