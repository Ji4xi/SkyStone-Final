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
        if (skystoneAngle > 0) {
            xAddOn = 6.9;
            YLinearInchesGyro(xAddOn, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        } else if (skystoneAngle == 0) {
            xAddOn = 6.9 + SKYSTONE_INCH;
            YLinearInchesGyro(xAddOn, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        } else {
            xAddOn = 6.9 + 2 * SKYSTONE_INCH;
            YLinearInchesGyro(xAddOn, GAGE.FORWARD, drivePwrMax - 0.3, -90);
        }

        double drift = 2;
        topClaw.setPosition(0.257);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(drift, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.33);
        XLinearInchesGyro(drift + 1.5, Side.RIGHT, drivePwrMax - 0.3, -90);

        if (skystoneAngle > 0)
            YLinearInchesGyro(16.9 + 3 * TILE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(16.9 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(16.9 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }

        XLinearInchesGyro(drift + 1.3, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0);
        topClaw.setPosition(0.4);
        sleep(200);
        XLinearInchesGyro(drift + 1.3, Side.RIGHT, drivePwrMax - 0.3, -90);
        //RAISE
        moveClaw(0, 0.33);

        if (skystoneAngle > 0)
            YLinearInchesGyro(-7.9 + 3 * TILE_INCH - 3 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(-7.9 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else {
            YLinearInchesGyro(-7.9 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        }

        //SECOND BLOCK
        topClaw.setPosition(0.257);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(drift + 2.7, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.33);
        XLinearInchesGyro(drift + 2.7, Side.RIGHT, drivePwrMax - 0.3, -90);

        if (skystoneAngle > 0)
            YLinearInchesGyro(-7.9 + 3 * TILE_INCH - 3 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(-7.9 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(-7.9 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }

        XLinearInchesGyro(drift + 1.4, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0);
        topClaw.setPosition(0.4);
        sleep(200);
        XLinearInchesGyro(drift + 1.4, Side.RIGHT, drivePwrMax - 0.3, -90);
        tape.setPosition(0.7);
        //RAISE
        moveClaw(0, 0.33);
    }

}
