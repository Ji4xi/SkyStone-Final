package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper.SkyScraper;

@Autonomous
public class StoneFinderIntakeBlue extends StoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        YLinearInchesGyro(19.2, GAGE.FORWARD, 0.40, 0);
        turnPID(-90, Direction.CLOCKWISE, 0.6);
        if (skystoneAngle > 0) {
            YLinearInchesGyro(3.3, GAGE.REVERSE, 0.40, 90);
        } else if (skystoneAngle < 0) {
            YLinearInchesGyro(3.3, GAGE.FORWARD, 0.40, -90);
        } else {
            YLinearInchesGyro(1.5, GAGE.REVERSE, 0.40, 90);
        }
        //moveInchesPID(10, skystoneAngle, 1000);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(2.4, Side.LEFT, 0.40, -90);
        topClaw.setPosition(0);
        sleep(300);
        moveClaw(0, 0.13);
        XLinearInchesGyro(5.4, Side.RIGHT, 0.40, -90);
        YLinearInchesGyro(56, GAGE.REVERSE, 0.50, 90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.13);
        if (skystoneAngle == 0)
            YLinearInchesGyro(78.5, GAGE.FORWARD, 0.65, -90); //DANGEROUS
        else if (skystoneAngle < 0) YLinearInchesGyro(77.5, GAGE.FORWARD, 0.40, -90); //DANGEROUS
        else {
            YLinearInchesGyro(75, GAGE.FORWARD, 0.70, -90); //DANGEROUS
        }
        //second brick
        bottomClaw.setPosition(0);
        XLinearInchesGyro(3.7, Side.LEFT, 0.50, -90);
        topClaw.setPosition(0);
        sleep(300);
        moveClaw(0, 0.15);
        XLinearInchesGyro(4.6, Side.RIGHT, 0.50, -90);
        YLinearInchesGyro(79.5, GAGE.REVERSE, 0.70, 90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.13);
        if (skystoneAngle == 0) {
            YLinearInchesGyro(19.5, GAGE.FORWARD, 0.60, -90);
        } else if (skystoneAngle > 0) {
            YLinearInchesGyro(25, GAGE.FORWARD, 0.70, -90);
        } else {
            YLinearInchesGyro(10, GAGE.FORWARD, 0.50, -90);
        }

    }

}
