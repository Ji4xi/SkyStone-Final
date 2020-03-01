package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.StoneFinder;

@Autonomous
public class SkystoneBlue extends MyOdometryOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
;

//
//        if (skystoneAngle > 0) {
//            public void runWithoutEncoders() {
//                fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//        } else if (skystoneAngle == 0) {
//            xAddOn = 3 * SKYSTONE_INCH - 5 + 2;
//            YLinearInchesGyro(xAddOn, StoneFinder.GAGE.REVERSE, drivePwrMax - 0.3, 90);
//        } else {
//            xAddOn = 2 * SKYSTONE_INCH - 4.5 + 3;
//            YLinearInchesGyro(xAddOn, StoneFinder.GAGE.REVERSE, drivePwrMax - 0.3, 90);
//        }
//
//        double drift = 2;
//        if (skystoneAngle == 0) {
//            XLinearInchesGyro(drift + 0.4, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        } else if (skystoneAngle > 0) {
//            XLinearInchesGyro(drift + 0.7, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        } else {
//            XLinearInchesGyro(drift + 0.4, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        }
//
//
//        topClaw.setPosition(0);
//        sleep(400);
//        //RAISE
//        moveClaw(0, 0.707);
//        if (skystoneAngle == 0) {
//            XLinearInchesGyro(drift + 2.5, StoneFinder.Side.RIGHT, drivePwrMax - 0.3, -90);
//        } else if (skystoneAngle > 0) {
//            XLinearInchesGyro(drift + 2.8, StoneFinder.Side.RIGHT, drivePwrMax - 0.3, -90);
//        }
//        else {
//            XLinearInchesGyro(drift + 2.2, StoneFinder.Side.RIGHT, drivePwrMax - 0.3, -90);
//        }
//
//        //need fix in distance travelled
//        if (skystoneAngle > 0)
//            YLinearInchesGyro(2 + 9.35 - 9.5 + 3 * TILE_INCH + SKYSTONE_INCH, StoneFinder.GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
//        else if (skystoneAngle == 0) YLinearInchesGyro(2 + 10.85 - 9.5 + 3 * TILE_INCH + 3 * SKYSTONE_INCH, StoneFinder.GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
//        else {
//            YLinearInchesGyro(3 + 10.85 - 9.5 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, StoneFinder.GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
//        }
//
//        if (skystoneAngle == 0 || skystoneAngle < 0) {
//            XLinearInchesGyro(drift + 2.7, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        } else {
//            XLinearInchesGyro(drift + 2.1, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        }
//
//        //DROP
//        bottomClaw.setPosition(0.332);
//        sleep(200);
//        topClaw.setPosition(0.48);
//        if (skystoneAngle == 0) XLinearInchesGyro(drift + 1.8, StoneFinder.Side.RIGHT , drivePwrMax - 0.3, -90);
//        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 2.3, StoneFinder.Side.RIGHT , drivePwrMax - 0.3, -90);
//        else XLinearInchesGyro(drift + 2.6, StoneFinder.Side.RIGHT , drivePwrMax - 0.3, -90);
//        //RAISE
//        moveClaw(0, 0.707);
//
//        //need fix in distance travelled
//        if (skystoneAngle > 0)
//            YLinearInchesGyro(-16.1 - 8.5 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, StoneFinder.GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
//        else if (skystoneAngle == 0) YLinearInchesGyro(-15.1 - 8.5 + 3 * TILE_INCH + 3 * SKYSTONE_INCH, StoneFinder.GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
//        else {
//            YLinearInchesGyro(-17.1 - 9.5 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, StoneFinder.GAGE.REVERSE, drivePwrMax + 0.3, 90); //DANGEROUS
//        }
//
//        //SECOND BLOCK
//        topClaw.setPosition(0.388);
//        bottomClaw.setPosition(0.332);
//        if (skystoneAngle > 0) XLinearInchesGyro(drift + 5.0, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 4.8, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        else XLinearInchesGyro(drift + 4.8, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//
//        topClaw.setPosition(0);
//        sleep(400);
//        //RAISE
//        moveClaw(0, 0.33);
//        if (skystoneAngle > 0) XLinearInchesGyro(drift + 5.8, StoneFinder.Side.RIGHT, drivePwrMax - 0.3, -90);
//        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 5.1, StoneFinder.Side.RIGHT, drivePwrMax - 0.3, -90);
//        else
//            XLinearInchesGyro(drift + 4.6, StoneFinder.Side.RIGHT, drivePwrMax - 0.3, -90);
//
//        //need fix in distance travelled
//        if (skystoneAngle > 0)
//            YLinearInchesGyro(0.9 - 11.5 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, StoneFinder.GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
//        else if (skystoneAngle == 0) YLinearInchesGyro(0.9 - 10.5 + 3 * TILE_INCH + 3 * SKYSTONE_INCH, StoneFinder.GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
//        else {
//            YLinearInchesGyro(0.9 - 12.5 + 3 * TILE_INCH + 2 * SKYSTONE_INCH, StoneFinder.GAGE.FORWARD, drivePwrMax + 0.3, -90); //DANGEROUS
//        }
//
//        if (skystoneAngle > 0) XLinearInchesGyro(drift + 6, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        else if (skystoneAngle < 0) XLinearInchesGyro(drift + 2.95 + 2.7, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        else XLinearInchesGyro(drift + 3.45, StoneFinder.Side.LEFT, drivePwrMax - 0.3, -90);
//        //DROP
//        bottomClaw.setPosition(0.332);
//        sleep(200);
//        topClaw.setPosition(0.48);
//        //Foundation
//        YLinearInchesGyro(1, StoneFinder.GAGE.FORWARD,drivePwrMax - 0.3, 180);
//        turnPID(-175, StoneFinder.Direction.CLOCKWISE, 0.6);//12
//        YLinearInchesGyro(3.4, StoneFinder.GAGE.REVERSE,drivePwrMax - 0.3, -180);
//        foundation(StoneFinder.Mode.OPEN, 100);
//        //extend servo
//        //tape.setPower(0.4);
//        if (skystoneAngle > 0) YLinearInchesGyro(TILE_INCH + 11, StoneFinder.GAGE.FORWARD, drivePwrMax , 180);
//        else YLinearInchesGyro(TILE_INCH + 11.2, StoneFinder.GAGE.FORWARD, drivePwrMax - 0.15, 180);
//        foundation(StoneFinder.Mode.CLOSE, 1500);
//        //turnPID(-70, Direction.CLOCKWISE, 0.2);//12
    }
}
