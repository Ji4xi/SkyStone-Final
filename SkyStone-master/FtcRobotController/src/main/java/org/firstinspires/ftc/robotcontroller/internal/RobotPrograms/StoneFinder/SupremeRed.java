package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class SupremeRed extends StoneFinder{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        double xAddOn = 0, yAddOn = 0;
        yAddOn = TILE_INCH;
        YLinearInchesGyro(yAddOn, GAGE.FORWARD, drivePwrMax - 0.3, 0); //0.4
        turnPID(-81, Direction.CLOCKWISE, 0.6);//12
        topClaw.setPosition(0.388);
        bottomClaw.setPosition(0.332);
        if (skystoneAngle > 0) {
            xAddOn = SKYSTONE_INCH - 4.5 + 2;
            YLinearInchesGyro(xAddOn, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        } else if (skystoneAngle == 0) {
            xAddOn = 3 * SKYSTONE_INCH - 5 + 2;
            YLinearInchesGyro(xAddOn, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        } else {
            xAddOn = 2 * SKYSTONE_INCH - 4.5 + 3;
            YLinearInchesGyro(xAddOn, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        }

        double drift = 2;
        if (skystoneAngle == 0) {
            XLinearInchesGyro(drift + 0.4, Side.LEFT, drivePwrMax - 0.3, -90);
        } else if (skystoneAngle > 0) {
            XLinearInchesGyro(drift + 0.7, Side.LEFT, drivePwrMax - 0.3, -90);
        } else {
            XLinearInchesGyro(drift + 0.4, Side.LEFT, drivePwrMax - 0.3, -90);
        }


        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.707);
        if (skystoneAngle == 0) {
            XLinearInchesGyro(drift + 2.5, Side.RIGHT, drivePwrMax - 0.3, -90);
        } else if (skystoneAngle > 0) {
            XLinearInchesGyro(drift + 2.8, Side.RIGHT, drivePwrMax - 0.3, -90);
        }
        else {
            XLinearInchesGyro(drift + 2.2, Side.RIGHT, drivePwrMax - 0.3, -90);
        }

        //need fix in distance travelled
        if (skystoneAngle > 0)
            YLinearInchesGyro(2 + 9.35 - 9.5 + 2 * TILE_INCH + SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(2 + 10.85 - 9.5 + 2 * TILE_INCH + 3 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else {
            YLinearInchesGyro(3 + 10.85 - 9.5 + 2 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        }

        if (skystoneAngle == 0 || skystoneAngle < 0) {
            XLinearInchesGyro(drift + 1.7, Side.LEFT, drivePwrMax - 0.3, -90);
        } else {
            XLinearInchesGyro(drift + 1.1, Side.LEFT, drivePwrMax - 0.3, -90);
        }

        //DROP
        bottomClaw.setPosition(0.332);
        sleep(200);
        topClaw.setPosition(0.48);
        if (skystoneAngle == 0) XLinearInchesGyro(drift + 0.8, Side.RIGHT , drivePwrMax - 0.3, -90);
        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 1.3, Side.RIGHT , drivePwrMax - 0.3, -90);
        else XLinearInchesGyro(drift + 1.6, Side.RIGHT , drivePwrMax - 0.3, -90);
        //RAISE
        moveClaw(0, 0.707);

        //need fix in distance travelled
        if (skystoneAngle > 0)
            YLinearInchesGyro(-16.1 - 8.5 + 2 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(-15.1 - 8.5 + 2 * TILE_INCH + 3 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(-17.1 - 9.5 + 2 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }

        //SECOND BLOCK
        topClaw.setPosition(0.388);
        bottomClaw.setPosition(0.332);
        if (skystoneAngle > 0) XLinearInchesGyro(drift + 5.0, Side.LEFT, drivePwrMax - 0.3, -90);
        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 4.8, Side.LEFT, drivePwrMax - 0.3, -90);
        else XLinearInchesGyro(drift + 4.8, Side.LEFT, drivePwrMax - 0.3, -90);

        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.707);
        if (skystoneAngle > 0) XLinearInchesGyro(drift + 5.8, Side.RIGHT, drivePwrMax - 0.3, -90);
        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 5.1, Side.RIGHT, drivePwrMax - 0.3, -90);
        else
            XLinearInchesGyro(drift + 4.6, Side.RIGHT, drivePwrMax - 0.3, -90);

        //need fix in distance travelled
        if (skystoneAngle > 0)
            YLinearInchesGyro(0.9 - 19.5 + 2 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(0.9 - 18.5 + 2 * TILE_INCH + 3 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else {
            YLinearInchesGyro(0.9 - 20.5 + 2 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        }

        if (skystoneAngle > 0) XLinearInchesGyro(drift + 6, Side.LEFT, drivePwrMax - 0.3, -90);
        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 2.95 + 2.7, Side.LEFT, drivePwrMax - 0.3, -90);
        else XLinearInchesGyro(drift + 3.45, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0.332);
        sleep(200);
        topClaw.setPosition(0.48);
        //Park
        XLinearInchesGyro(drift + 0.7 - 1.4, Side.RIGHT, drivePwrMax - 0.3, -90);
        YLinearInchesGyro(0.5 * TILE_INCH - 7, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS

    }
}
