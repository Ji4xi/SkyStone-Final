package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

public class NewSkyStoneIntakeRed extends StoneFinder{
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
            xAddOn = 0;
            YLinearInchesGyro(xAddOn, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        } else if (skystoneAngle == 0) {
            xAddOn = 0;
            YLinearInchesGyro(xAddOn, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        } else {
            xAddOn = SKYSTONE_INCH;
            YLinearInchesGyro(xAddOn, GAGE.REVERSE, drivePwrMax - 0.3, 90);
        }

        double drift = 2;
        XLinearInchesGyro(drift, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.33);
        XLinearInchesGyro(drift + 1.8, Side.RIGHT, drivePwrMax - 0.3, -90);

        //need fix in distance travelled
        if (skystoneAngle > 0)
            YLinearInchesGyro(17.85 + 3 * TILE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(17.85 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        else {
            YLinearInchesGyro(17.85 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
        }

        XLinearInchesGyro(drift + 1.9, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0);
        topClaw.setPosition(0.4);
        sleep(200);
        XLinearInchesGyro(drift + 1.1, Side.RIGHT , drivePwrMax - 0.3, -90);
        //RAISE
        moveClaw(0, 0.33);

        //need fix in distance travelled
        if (skystoneAngle > 0)
            YLinearInchesGyro(-7.1 + 3 * TILE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(-7.1 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(-7.1 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }

        //SECOND BLOCK
        topClaw.setPosition(0.257);
        bottomClaw.setPosition(0);
        XLinearInchesGyro(drift + 1.7, Side.LEFT, drivePwrMax - 0.3, -90);
        topClaw.setPosition(0);
        sleep(400);
        //RAISE
        moveClaw(0, 0.33);
        XLinearInchesGyro(drift + 2.8, Side.RIGHT, drivePwrMax - 0.3, -90);

        //need fix in distance travelled
        if (skystoneAngle > 0)
            YLinearInchesGyro(0.9 + 3 * TILE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else if (skystoneAngle == 0) YLinearInchesGyro(0.9 + 3 * TILE_INCH + SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        else {
            YLinearInchesGyro(0.9 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
        }

        XLinearInchesGyro(drift + 1.9, Side.LEFT, drivePwrMax - 0.3, -90);
        //DROP
        bottomClaw.setPosition(0);
        topClaw.setPosition(0.4);

        //Foundation
        YLinearInchesGyro(.5, GAGE.REVERSE,drivePwrMax - 0.3, -180);
        turnPID(-175, Direction.CLOCKWISE, 0.6);//12
        YLinearInchesGyro(2, GAGE.FORWARD,drivePwrMax - 0.3, 180);
        foundation(Mode.OPEN, 100);
        //extend servo
        //tape.setPower(0.4);
        YLinearInchesGyro(TILE_INCH + 4.5, GAGE.REVERSE,drivePwrMax - 0.3, -180);
        foundation(Mode.CLOSE, 0);
        //tape.setPower(0);
    }
}
