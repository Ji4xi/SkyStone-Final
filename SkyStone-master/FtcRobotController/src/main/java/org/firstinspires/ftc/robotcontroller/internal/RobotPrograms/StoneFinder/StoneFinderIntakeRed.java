package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class StoneFinderIntakeRed extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        YLinearInchesGyro(20.3, GAGE.FORWARD, 0.40, 0);
        turnPID(-90, Direction.CLOCKWISE, 0.6);
        if (skystoneAngle < 0) {
            YLinearInchesGyro(3, GAGE.FORWARD, 0.40, -90);
        } else if (skystoneAngle > 0) {
            YLinearInchesGyro(3, GAGE.REVERSE, 0.40, 90);
        }
        //moveInchesPID(10, skystoneAngle, 1000);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(1.8, Side.LEFT, 0.40, -90);
        topClaw.setPosition(0);
        sleep(300);
        moveClaw(0, 0.15);//0.3
        XLinearInchesGyro(6, Side.RIGHT, 0.40, -90);//4
        YLinearInchesGyro(56, GAGE.FORWARD, 0.50, -90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.15);//0.3
        if (skystoneAngle == 0)
        YLinearInchesGyro(78.5, GAGE.REVERSE, 0.40, 90); //DANGEROUS
        else if (skystoneAngle < 0) YLinearInchesGyro(77.5, GAGE.REVERSE, 0.40, 90); //DANGEROUS
        else {
            YLinearInchesGyro(65.5, GAGE.REVERSE, 0.40, 90); //DANGEROUS
        }
        //second brick
        bottomClaw.setPosition(0);
        XLinearInchesGyro(4, Side.LEFT, 0.50, -90);
        topClaw.setPosition(0);
        sleep(300);
        moveClaw(0, 0.15);//0.15
        XLinearInchesGyro(4, Side.RIGHT, 0.50, -90);
        YLinearInchesGyro(78, GAGE.FORWARD, 0.50, -90);

        moveClaw(0, 0.05);
        sleep(300);
        moveClaw(0.5, 0.15);//0.15
        XLinearInchesGyro(2, Side.LEFT, 0.40, -90);//4
        YLinearInchesGyro(10, GAGE.REVERSE, 0.50, 90);
    }
}
