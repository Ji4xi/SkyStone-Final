package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper.SkyScraper;

@Autonomous
public class NewSkyStoneIntakeBlue extends StoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        double xAddOn = 0, yAddOn = 0;
        yAddOn = TILE_INCH;
        YLinearInchesGyro(yAddOn, GAGE.FORWARD, drivePwrMax - 0.3, 0); //0.4
        turnPID(-81, Direction.CLOCKWISE, 0.6);//12
        topClaw.setPosition(0.257);
        bottomClaw.setPosition(0);
        if (skystoneAngle > 0) {
            xAddOn = 7.35;
            YLinearInchesGyro(xAddOn, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        } else if (skystoneAngle == 0) {
            xAddOn = 7.35 + SKYSTONE_INCH;
            YLinearInchesGyro(xAddOn, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        } else {
            xAddOn = 7.35 + 2 * SKYSTONE_INCH;
            YLinearInchesGyro(xAddOn, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        }

        double drift = 2;
        XLinearInchesGyro(drift, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.33);
        XLinearInchesGyro(drift + 1.8, Side.RIGHT, drivePwrMax - 0.3, -90);

        if (skystoneAngle > 0)
            YLinearInchesGyro(17.85 + 3 * TILE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(18.85 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(19.85 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }


        if (skystoneAngle < 0) XLinearInchesGyro(drift + 2.7, Side.LEFT, drivePwrMax - 0.3, -90);
            else XLinearInchesGyro(drift + 1.9, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0);
        topClaw.setPosition(0.4);
        sleep(200);
        if (skystoneAngle < 0) XLinearInchesGyro(drift + 1.9, Side.RIGHT , drivePwrMax - 0.3, -90);
        else XLinearInchesGyro(drift + 1.1, Side.RIGHT , drivePwrMax - 0.3, -90);
        //RAISE
        moveClaw(0, 0.33);

        if (skystoneAngle > 0)
            YLinearInchesGyro(-7.1 + 3 * TILE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(-6.1 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else {
            YLinearInchesGyro(-4.1 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        }

        //SECOND BLOCK
        topClaw.setPosition(0.257);
        bottomClaw.setPosition(0);
        if (skystoneAngle < 0) XLinearInchesGyro(drift + 4.1, Side.LEFT, drivePwrMax - 0.3, -90);
        else XLinearInchesGyro(drift + 3.6, Side.LEFT, drivePwrMax - 0.3, -90);

        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.33);
        if (skystoneAngle < 0) XLinearInchesGyro(drift + 5.2, Side.RIGHT, drivePwrMax - 0.3, -90);
            else XLinearInchesGyro(drift + 4.7, Side.RIGHT, drivePwrMax - 0.3, -90);


        if (skystoneAngle > 0)
            YLinearInchesGyro(0.9 + 3 * TILE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(0.9 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(1.9 - 4 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }


        if (skystoneAngle < 0) XLinearInchesGyro(drift + 4.7, Side.LEFT, drivePwrMax - 0.3, -90);
            else XLinearInchesGyro(drift + 3.1, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0);
        topClaw.setPosition(0.4);

        //extend servo

        //Foundation
        YLinearInchesGyro(.5, GAGE.FORWARD,drivePwrMax - 0.3, 180);
        turnPID(-175, Direction.CLOCKWISE, 0.6);//12
        YLinearInchesGyro(3, GAGE.REVERSE,drivePwrMax - 0.3, -180);
        foundation(Mode.OPEN, 100);
        //tape.setPower(0.4);
        YLinearInchesGyro(TILE_INCH + 9.8, GAGE.FORWARD,drivePwrMax - 0.3, 180);
        foundation(Mode.CLOSE, 1500);

        //tape.setPower(0);

    }

}
