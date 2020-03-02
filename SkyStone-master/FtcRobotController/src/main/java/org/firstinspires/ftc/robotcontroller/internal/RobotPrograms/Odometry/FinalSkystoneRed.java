package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcontroller.internal.Default.PID;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

import java.util.ServiceLoader;
@Disabled
@Autonomous
public class FinalSkystoneRed extends NewStoneFinder {

    int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        maxPwr = 1;
        super.runOpMode();

        //bot 0.39 level to grab, 0.75 vertical up
        //top
        double skystone_pos;
        if (skystoneAngle < 0) {
            skystone_pos = -(SKYSTONE_INCH * 1 + 10);
        } else if (skystoneAngle == 0) {
            skystone_pos = -(SKYSTONE_INCH * 3 + 10);
        } else {
            skystone_pos = -(SKYSTONE_INCH * 2 + 10);
        }
        goToPositionSupreme(skystone_pos, TILE_INCH + 6, maxPwr, 90, 1);
//
//        //open up
        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);
        sleep(200);
        if (skystoneAngle < 0) {

            turnPID(0, Direction.CLOCKWISE, maxPwr+0.65);

        } else {

            turnPID(0, Direction.CLOCKWISE, maxPwr+0.65);

        }
        goToPositionHard(skystone_pos, TILE_INCH + 8, maxPwr-0.6, 0, 1);
        topClaw.setPosition(0.855);
        sleep(500);
        bottomClaw.setPosition(0.166);
        goToPositionHard(skystone_pos, TILE_INCH + 4, maxPwr-0.6, 0, 2);
        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4, TILE_INCH + 4, maxPwr, 0, 2);

        goToPositionHard(skystone_pos + 4 * TILE_INCH-4, TILE_INCH + 10, maxPwr-0.6, 0, 2);
        bottomClaw.setPosition(0.500);
        sleep(100);
        topClaw.setPosition(0.2329);

        goToPositionHard(skystone_pos + 4 * TILE_INCH-4, TILE_INCH + 8, maxPwr-0.6, 0, 1);
        bottomClaw.setPosition(0.167);
        topClaw.setPosition(0.855);

        skystone_pos+=24;
        goToPositionSupreme(skystone_pos, TILE_INCH + 6, maxPwr, 0, 1);

        goToPositionHard(skystone_pos, TILE_INCH + 8, maxPwr-0.6, 0, 1);
        topClaw.setPosition(0.855);
        sleep(500);
        bottomClaw.setPosition(0.166);
        goToPositionHard(skystone_pos, TILE_INCH + 4, maxPwr-0.6, 0, 2);

        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4-12, TILE_INCH + 4, maxPwr, 0, 2);

        goToPositionHard(skystone_pos + 4 * TILE_INCH-4-12, TILE_INCH + 10, maxPwr-0.6, 0, 2);
        bottomClaw.setPosition(0.500);
        sleep(100);
        topClaw.setPosition(0.2329);

        goToPositionHard(skystone_pos + 4 * TILE_INCH-12, TILE_INCH + 8, maxPwr-0.6, 0, 1);
        bottomClaw.setPosition(0.167);
        topClaw.setPosition(0.855);


//        topClaw.setPosition(0.34);
//        bottomClaw.setPosition(0.617);
//        deliver(skystone_pos, 0);
//        double brickOffset = 24;
//        skystone_pos+=brickOffset;
//        if (skystoneAngle < 0) {
//
//            turnPID(270, Direction.CLOCKWISE, maxPwr+0.9);
//
//            goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4, 47.25-17.5+4, maxPwr, 270, 1.5);
//            //foundation
//            ls.setPosition(0.5);
//            rs.setPosition(0.5);
//            sleep(300);
//            goToPositionSupreme(skystone_pos + 4 *  TILE_INCH-4, 47.25-17.5+4 - 0.5 * TILE_INCH, maxPwr, 270, 1);
//
//
//            turnPID(180, Direction.CLOCKWISE, maxPwr+0.9);
//
//            ls.setPosition(0);
//            rs.setPosition(0);
//            goToPositionSupreme(skystone_pos - 3 * SKYSTONE_INCH, 47.25-17.5+4 - 0.5 * TILE_INCH, maxPwr, 180, 1);
//
//            turnPID(135, Direction.CLOCKWISE, maxPwr+0.9);
//            hook.setPosition(0.8);
//            leftIntake.setPower(0.4);
//            rightIntake.setPower(0.4);
//            goToPositionSupreme(skystone_pos - 3 * SKYSTONE_INCH+TILE_INCH, 47.25-17.5+4 + 0.5 * TILE_INCH, maxPwr, 180, 1);
//            goToPositionSupreme(skystone_pos - 3 * SKYSTONE_INCH, 47.25-17.5+4 - 0.5 * TILE_INCH, maxPwr, 180, 1);
//            leftIntake.setPower(0);
//            rightIntake.setPower(0);
//        } else {
//            goToPositionSupreme(skystone_pos, TILE_INCH +12, maxPwr+0.1, 0, 1.5);
//            deliver(skystone_pos, -brickOffset +SKYSTONE_INCH*1.5);
//
//            turnPID(270, Direction.CLOCKWISE, maxPwr+0.6);
//
//            goToPositionHard(skystone_pos + 4 * TILE_INCH-4 + -brickOffset+SKYSTONE_INCH*1.5, 47.25-17.5+8, maxPwr-0.6+0.2, 270, 1);
//            //foundation
//            ls.setPosition(0.5);
//            rs.setPosition(0.5);
//            sleep(300);
//            goToPositionHard(TILE_INCH*5-17.5-2, 4, maxPwr, 270, 3);
//            ls.setPosition(0);
//            rs.setPosition(0);
//            sleep(100);
//            goToPositionSupreme(TILE_INCH*2-17.5, 4, maxPwr, 270, 1);
//        }



    }
    public void deliver(double skystone_pos, double offset) throws InterruptedException{

        //open up
        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);
        sleep(100);
        goToPositionHard(skystone_pos, TILE_INCH + 10, maxPwr-0.7+0.2, 0, 3);
        //globalCoordinate.setGlobalX(globalCoordinate.getGlobalX() + 2 * SKYSTONE_INCH);


        topClaw.setPosition(0.855);
        sleep(500);
        bottomClaw.setPosition(0.166);
        goToPositionHard(skystone_pos+5, TILE_INCH + 4 + count*3, maxPwr-0.7+0.2, 0, 3);


//        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 3.2, 0.9, 0, 2);

        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 17, maxPwr+0.15, 0, 1.5);
        bottomClaw.setPosition(0.500);
        sleep(100);
        topClaw.setPosition(0.2329);

        goToPositionHard(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 12, maxPwr-0.6+0.2, 0, 3);
        bottomClaw.setPosition(0.167);
        topClaw.setPosition(0.855);
        count++;
    }
}