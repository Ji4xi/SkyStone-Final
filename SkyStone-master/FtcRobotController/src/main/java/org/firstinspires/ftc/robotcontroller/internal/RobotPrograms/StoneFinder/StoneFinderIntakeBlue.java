package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper.SkyScraper;

@Autonomous
public class StoneFinderIntakeBlue extends StoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        YLinearInchesGyro(16.8, GAGE.FORWARD, 0.6, 0);//0.4
        turnPID(-90, Direction.CLOCKWISE, 0.5);
        if (skystoneAngle > 0) {
            YLinearInchesGyro(4, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        } else if (skystoneAngle < 0) {
            YLinearInchesGyro(3.3, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        } else {
            YLinearInchesGyro(1.5, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        }
        //moveInchesPID(10, skystoneAngle, 1000);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(3.4, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0.035);
        sleep(400);
        moveClaw(0, 0.13);
        XLinearInchesGyro(9.6, Side.RIGHT, drivePwrMax - 0.3, -90);
        YLinearInchesGyro(32, GAGE.REVERSE, drivePwrMax, 90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.08); //0.13
        if (skystoneAngle == 0)
            YLinearInchesGyro(54.5, GAGE.FORWARD, drivePwrMax, -90); //DANGEROUS
        else if (skystoneAngle < 0) YLinearInchesGyro(53.5, GAGE.FORWARD, drivePwrMax, -90); //DANGEROUS
        else {
            YLinearInchesGyro(51, GAGE.FORWARD, drivePwrMax, -90); //DANGEROUS
        }
        //second brick
        bottomClaw.setPosition(0);
        XLinearInchesGyro(4.8, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0.035);
        sleep(400);
        moveClaw(0, 0.13);
        XLinearInchesGyro(10.35, Side.RIGHT, drivePwrMax - 0.3, -90);
        YLinearInchesGyro(69.5, GAGE.REVERSE, drivePwrMax, 90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.08); //0.13
        if (skystoneAngle == 0) {
            YLinearInchesGyro(31.5, GAGE.FORWARD, drivePwrMax, -90);
        } else if (skystoneAngle > 0) {
            YLinearInchesGyro(37, GAGE.FORWARD, drivePwrMax, -90);
        } else {
            YLinearInchesGyro(22, GAGE.FORWARD, drivePwrMax, -90);
        }

        XLinearInchesGyro(4, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        bottomClaw.setPosition(0.0157);

    }

}
