package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class StoneFinderIntakeRed extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        YLinearInchesGyro(16.8, GAGE.FORWARD, 0.60, 0);
        turnPID(-90, Direction.CLOCKWISE, 0.5);
        if (skystoneAngle < 0) {
            YLinearInchesGyro(3, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        } else if (skystoneAngle > 0) {
            YLinearInchesGyro(3, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        }
        //moveInchesPID(10, skystoneAngle, 1000);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(3.4, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0.035);
        sleep(400);
        moveClaw(0, 0.13);//0.3
        XLinearInchesGyro(9.6, Side.RIGHT, drivePwrMax, -90);//4
        YLinearInchesGyro(54, GAGE.FORWARD, drivePwrMax, -90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.8);//0.3
        if (skystoneAngle == 0)
        YLinearInchesGyro(76.5, GAGE.REVERSE, drivePwrMax, 90); //DANGEROUS
        else if (skystoneAngle < 0) YLinearInchesGyro(75.5, GAGE.REVERSE, drivePwrMax, 90); //DANGEROUS
        else {
            YLinearInchesGyro(63.5, GAGE.REVERSE, drivePwrMax, 90); //DANGEROUS
        }
        //second brick
        bottomClaw.setPosition(0);
        XLinearInchesGyro(4, Side.LEFT, 0.50, -90);
        topClaw.setPosition(0.035);
        sleep(400);
        moveClaw(0, 0.13);//0.15
        XLinearInchesGyro(9.35, Side.RIGHT, 0.50, -90);
        YLinearInchesGyro(72, GAGE.FORWARD, 0.50, -90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.08);//0.15
        XLinearInchesGyro(2, Side.LEFT, 0.40, -90);//4
        YLinearInchesGyro(4, GAGE.REVERSE, 0.50, 90);

        topClaw.setPosition(0);
        bottomClaw.setPosition(0.157);
    }
}
