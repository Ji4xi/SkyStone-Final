package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.MichaelBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.MichaelBot.MichaelBot;

@Autonomous
public class MichaelBotFullAutoRed extends MichaelBot {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        //from left to right = 33, 25, 17, 9, 1, -7
        YLinearInchesGyro(6, GAGE.FORWARD, drivePwrMax, 0);
        turnPID(-90, Direction.CLOCKWISE, 0.5);
        if (skystoneAngle <= 52) { //from left to right = 52, 63, 71, 80, 89, -82
        }
        if (skystoneAngle >= 52|| skystoneAngle <= 63) {
        }
        if (skystoneAngle >= 63|| skystoneAngle <= 71) {
        }
        if (skystoneAngle >= 71|| skystoneAngle <= 80) {
        }
        if (skystoneAngle >= 80|| skystoneAngle <= 89) {
        }
        if (skystoneAngle >= 89) {
        }
    }
}
