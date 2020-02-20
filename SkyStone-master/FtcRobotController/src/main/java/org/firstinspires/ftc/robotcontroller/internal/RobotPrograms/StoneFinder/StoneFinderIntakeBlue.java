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
        topClaw.setPosition(0.388);
        bottomClaw.setPosition(0.332);
        XLinearInchesGyro(3.4, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        sleep(400);
        moveClaw(0, 0.707);
        if (skystoneAngle > 0) {
            XLinearInchesGyro(8.5, Side.RIGHT, drivePwrMax - 0.3, -90);
        } else {
            XLinearInchesGyro(7.6, Side.RIGHT, drivePwrMax - 0.3, -90);
        }

        YLinearInchesGyro(32.6, GAGE.REVERSE, drivePwrMax, 90);

        moveClaw(0, 0.355);
        sleep(300);
        moveClaw(0.48, 0.707); //0.13
        if (skystoneAngle == 0)
            YLinearInchesGyro(54.5 + 1, GAGE.FORWARD, drivePwrMax, -90); //DANGEROUS
        else if (skystoneAngle < 0) YLinearInchesGyro(53.5 + 0.6, GAGE.FORWARD, drivePwrMax, -90); //DANGEROUS
        else {
            YLinearInchesGyro(51 - 3 + 1.1, GAGE.FORWARD, drivePwrMax, -90); //DANGEROUS
        }
        //second brick
        topClaw.setPosition(0.388);
        bottomClaw.setPosition(0.332);
        if (skystoneAngle < 0)  XLinearInchesGyro(6.0 + 0.2, Side.LEFT, drivePwrMax - 0.3, -90);
        else if (skystoneAngle > 0) XLinearInchesGyro(6.5 + 0.6, Side.LEFT, drivePwrMax - 0.3, -90);
        else {
            XLinearInchesGyro(5.4, Side.LEFT, drivePwrMax - 0.3, -90);
        }

        topClaw.setPosition(0);
        sleep(400);
        moveClaw(0, 0.707);
        if (skystoneAngle < 0)  XLinearInchesGyro(10.95 + 0.2, Side.RIGHT, drivePwrMax - 0.3, -90);
        else {
            XLinearInchesGyro(10.35, Side.RIGHT, drivePwrMax - 0.3, -90);
        }

        if (skystoneAngle > 0) {
            YLinearInchesGyro(60, GAGE.REVERSE, drivePwrMax, 90);
        } else if (skystoneAngle == 0) {
            YLinearInchesGyro(59.9, GAGE.REVERSE, drivePwrMax, 90);
        }
        else {
            YLinearInchesGyro(59.5, GAGE.REVERSE, drivePwrMax, 90);
        }

        moveClaw(0, 0.355);
        sleep(300);
        moveClaw(0.48, 0.707); //0.13

        if (skystoneAngle == 0) {
            YLinearInchesGyro(21.5, GAGE.FORWARD, drivePwrMax, -90);
        } else if (skystoneAngle > 0) {
            YLinearInchesGyro(27, GAGE.FORWARD, drivePwrMax, -90);
        } else {
            YLinearInchesGyro(12 - 3, GAGE.FORWARD, drivePwrMax, -90);
        }

        if (skystoneAngle > 0) {
            XLinearInchesGyro(19, Side.LEFT, drivePwrMax, -90);
        } else if (skystoneAngle == 0) XLinearInchesGyro(16, Side.LEFT, drivePwrMax, -90);
        else {
            XLinearInchesGyro(14, Side.LEFT, drivePwrMax, -90);
        }
        topClaw.setPosition(0);
        bottomClaw.setPosition(0.0157);

    }

}