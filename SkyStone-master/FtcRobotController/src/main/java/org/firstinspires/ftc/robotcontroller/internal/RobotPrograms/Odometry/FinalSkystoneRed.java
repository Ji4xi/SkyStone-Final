package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcontroller.internal.Default.PID;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

import java.util.ServiceLoader;
//@Disabled
@Autonomous
public class FinalSkystoneRed extends NewStoneFinder {

    int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        maxPwr = 0.75; //0.75
        super.runOpMode();

        //bot 0.39 level to grab, 0.75 vertical up
        //top
        double skystone_pos;
        if (skystoneAngle < 0) {
            skystone_pos = -(SKYSTONE_INCH * 1 + 10);
        } else if (skystoneAngle == 0) {
            skystone_pos = -(SKYSTONE_INCH * 2 + 10);
        } else {
            skystone_pos = -(SKYSTONE_INCH * 2 + 12);
        }

        if (skystoneAngle > 0) {
            goToPositionSupreme(skystone_pos + 0.5 - 2, TILE_INCH + 4, maxPwr, 90, 1.3, 0.001035, 0, 0);
        } else {
            goToPositionSupreme(skystone_pos + 0.5, TILE_INCH + 4, maxPwr, 90, 1.3, 0.001035, 0, 0);
        }

        if (skystoneAngle == 0) {
            skystone_pos = -(SKYSTONE_INCH * 3 + 10);

        }


//
//        //open up

        if (skystoneAngle < 0) {
            turnPID(0, Direction.CLOCKWISE, maxPwr, 0.0180, 0.01, 0.001,4.5);
        } else {

            turnPID(0, Direction.CLOCKWISE, maxPwr, 0.0180, 0.01, 0.001,4.5);

        }

        if (skystoneAngle == 0) {
            skystone_pos -= 3;
            goToPositionSupreme(skystone_pos, TILE_INCH + 6, 0.6, 0, 1.3, 0.00262, 0, 0);

        }

        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);

        goToPositionSupreme(skystone_pos + 0.5, TILE_INCH + 12.4, maxPwr, 0, 1.2,0.00240, 0.0, 0.0001);//.0025
        topClaw.setPosition(0.855);
        sleep(300);
        bottomClaw.setPosition(0.166);
        goToPositionSupreme(skystone_pos + 0.5, TILE_INCH + 3.5, maxPwr, 0, 1.2,0.00175, 0.0, 0.0001);
        if (skystoneAngle == 0) {
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7, TILE_INCH + 2.8, 0.9, 0, 1.55,0.00156, 0, 0.00034); //d: 0.00033
        } else if (skystoneAngle < 0) {
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36, TILE_INCH + 2.8, 0.9, 0, 1.56,0.00155, 0, 0.00034); //d: 0.00033
        }
        else {
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7, TILE_INCH + 6.8, 0.9, 0, 1.55,0.0015, 0, 0.00034); //d: 0.00033
        }



//
        if (skystone_pos != 100) {

            if (skystoneAngle > 0) {
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10) + 4 * TILE_INCH-5, TILE_INCH + 14.8, maxPwr, 0, 1.3,0.0021, 0.0, 0.000145);
            } else  {
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10) + 4 * TILE_INCH-5, TILE_INCH + 13.8, maxPwr, 0, 1.3,0.0021, 0.0, 0.000145);
            }

            sleep(150);
            bottomClaw.setPosition(0.50);
            sleep(100);
            topClaw.setPosition(0.2329);
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10) + 4 * TILE_INCH-7, TILE_INCH + 9, maxPwr, 0, 1,0.0021, 0.0, 0.0001);
//
//        goToPositionHard(skystone_pos + 4 * TILE_INCH-4, TILE_INCH + 8, maxPwr-0.6, 0, 1);
            bottomClaw.setPosition(0.21);
            topClaw.setPosition(0.47);
            if (skystone_pos < 0) {
                skystone_pos+=24;
            } else if (skystone_pos > 0) {
                skystone_pos-=0;
            }
            else {
                skystone_pos-=16;
            }

            goToPositionSupreme(skystone_pos - 0, TILE_INCH + 9, 0.9, 0,1.5,0.001596, 0, 0.00042); //d:0.00155
            topClaw.setPosition(0.16);
            bottomClaw.setPosition(0.617);
            goToPositionSupreme(skystone_pos - 0, TILE_INCH + 15.6, maxPwr, 0, 1,0.00250, 0.0, 0.0001);
            topClaw.setPosition(0.855);
            sleep(300);
            bottomClaw.setPosition(0.166);
            goToPositionSupreme(skystone_pos - 1, TILE_INCH + 6.5, maxPwr, 0, 1.35,0.00164, 0.0, 0); //0.0017
            if (skystoneAngle <= 0) {
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+24 + 4 * TILE_INCH -7 + 12-22.5-3-3, TILE_INCH + 5.5, 0.9, 0, 1.7,0.00143, 0, 0.000285); //0.00140
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+24 + 4 * TILE_INCH -7 + 12-22.5 -3-3, TILE_INCH + 14.4, maxPwr, 0, 1.2,0.0018, 0.0, 0.000);//0.0020
            } else if (skystoneAngle > 0) {
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+24 + 4 * TILE_INCH -7 + 12-22.5-3-3 + 10, TILE_INCH + 9.5, 0.9, 0, 1.7,0.00143, 0, 0.000285); //0.00140
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+24 + 4 * TILE_INCH -7 + 12-22.5 -3-3 + 10, TILE_INCH + 18.4, maxPwr, 0, 1.2,0.0018, 0.0, 0.000);//0.0020
            }

            sleep(150);
            bottomClaw.setPosition(0.50);
            sleep(100);
            topClaw.setPosition(0.2329);

            turnPID(270, Direction.CLOCKWISE, maxPwr, 0.019, 0.01, 0.001);
            topClaw.setPosition(0.16);
            bottomClaw.setPosition(0.617);
            if (skystoneAngle > 0) {
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+24 + 4 * TILE_INCH-7+12-27-3+10, 47.25-17.5+12.8, maxPwr, 270, 1.5,0.00215,0,0.0001);
            } else {
                goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+24 + 4 * TILE_INCH-7+12-27-3, 47.25-17.5+11.8, maxPwr, 270, 1.5,0.0021,0,0.0001);
            }

            //foundation
            ls.setPosition(0.5);
            rs.setPosition(0.5);
            sleep(300);
//            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36, TILE_INCH + 3.9, maxPwr, 270, 1.5,0.001799, 0, 0.00035);
//            turnPID(180, Direction.CLOCKWISE, 0.85, 0.023, 0.10, 0.00, 4.5);
//            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36+2, TILE_INCH +1.4, 0.88, 180, 1.4,0.0022,0,0.000);

            bottomClaw.setPosition(0.21);
            topClaw.setPosition(0.47);
//            ls.setPosition(0);
//            rs.setPosition(0);
//            sleep(300);
//            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36 - 3 * TILE_INCH, TILE_INCH + 3.9, maxPwr, 180, 1.5,0.001799, 0, 0.00035);
//
            if (skystoneAngle > 0) {

            } else {

            }
            goToPositionSupreme(TILE_INCH*5-17.5-2, 0.5, 0.9, 270, 1.9,1.5);
            ls.setPosition(0);
            rs.setPosition(0);
            sleep(100);
            goToPositionSupreme(TILE_INCH*2-17.5, 0.7, 0.9, 270, 1.5);
        } else {

            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36, TILE_INCH + 12.5, maxPwr, 0, 1.3,0.0020, 0.0, 0.000172);
            sleep(150);
            bottomClaw.setPosition(0.50);
            sleep(100);
            topClaw.setPosition(0.2329);
            bottomClaw.setPosition(0.617);
            turnPID(270, Direction.CLOCKWISE, 0.85, 0.023, 0.10, 0.00, 4.5);
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36, 47.25-17.5+11.8, maxPwr, 270, 1.25,0.00167,0,0.000);
            //foundation
            ls.setPosition(0.5);
            rs.setPosition(0.5);
            sleep(300);
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36, TILE_INCH + 0.9, maxPwr, 270, 1.5,0.001799, 0, 0.00035);
            turnPID(180, Direction.CLOCKWISE, 0.85, 0.023, 0.10, 0.00, 4.5);
//            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36+2, TILE_INCH +1.4, 0.88, 180, 1.4,0.0022,0,0.000);
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 35)+ 4 * TILE_INCH-7 + 36 + 4, TILE_INCH + 0.9, maxPwr, 180, 1.5,0.001799, 0, 0.00035);
            bottomClaw.setPosition(0.21);
            topClaw.setPosition(0.47);
            ls.setPosition(0);
            rs.setPosition(0);
            sleep(300);
            skystone_pos -= 3.5; // -2.1
            goToPositionSupreme(skystone_pos + 3, TILE_INCH + 1.4, 0.9, 180,1.5,0.001550, 0, 0.000398); //d:0.00155
            rightIntake.setPower(1);
            leftIntake.setPower(1);
            turnPID(120, Direction.CLOCKWISE,  maxPwr, 0.019, 0.01, 0.001, 4.1);
            hook.setPosition(0.27);
            goToPositionSupreme(skystone_pos - 3.5, TILE_INCH + 10.4, maxPwr, 120,1.4,0.0020, 0.0, 0.000); //d:0.00155
            sleep(100);
            goToPositionSupreme(skystone_pos + 2, TILE_INCH + 1.4 + 2.5, maxPwr, 120,1.2,0.00155, 0.0, 0.000); //d:0.00155
            hook.setPosition(0.4);
            rightIntake.setPower(0);
            leftIntake.setPower(0);
            goToPositionSupreme(skystone_pos + 3.7*TILE_INCH + 3, TILE_INCH + 1.4 + 2.5, 0.9, 120,1.8,0.00165, 0, 0.00039); //d:0.00155
            goToPositionSupreme(skystone_pos + 3*TILE_INCH, TILE_INCH + 1.4 + 2.5, maxPwr, 120,1.2,0.00158, 0, 0.00039); //d:0.00155


        }

    }
    public void deliver(double skystone_pos, double offset) throws InterruptedException{
//
//        //open up
//        topClaw.setPosition(0.16);
//        bottomClaw.setPosition(0.617);
//        sleep(100);
//        goToPositionHard(skystone_pos, TILE_INCH + 10, maxPwr-0.7+0.2, 0, 3);
//        //globalCoordinate.setGlobalX(globalCoordinate.getGlobalX() + 2 * SKYSTONE_INCH);
//
//
//        topClaw.setPosition(0.855);
//        sleep(500);
//        bottomClaw.setPosition(0.166);
//        goToPositionHard(skystone_pos+5, TILE_INCH + 4 + count*3, maxPwr-0.7+0.2, 0, 3);
//
//
////        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 3.2, 0.9, 0, 2);
//
//        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 17, maxPwr+0.15, 0, 1.5);
//        bottomClaw.setPosition(0.500);
//        sleep(100);
//        topClaw.setPosition(0.2329);
//
//        goToPositionHard(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 12, maxPwr-0.6+0.2, 0, 3);
//        bottomClaw.setPosition(0.167);
//        topClaw.setPosition(0.855);
//        count++;
    }
}