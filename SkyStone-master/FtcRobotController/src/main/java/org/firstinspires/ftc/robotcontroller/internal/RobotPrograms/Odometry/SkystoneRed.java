package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcontroller.internal.Default.PID;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

import java.util.ServiceLoader;

@Autonomous
public class SkystoneRed extends NewStoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {

    maxPwr = 0.75;

        super.runOpMode();

        //bot 0.39 level to grab, 0.75 vertical up
        //top
        double skystone_pos;
        if (skystoneAngle < 0) {
            skystone_pos = -(SKYSTONE_INCH * 2 + 6);
        } else if (skystoneAngle == 0) {
            skystone_pos = -(SKYSTONE_INCH * 2 + 6);
        } else {
            skystone_pos = -(SKYSTONE_INCH * 2 + 6);
        }
        goToPositionSupreme(skystone_pos, TILE_INCH + 3.2, maxPwr, 90, 1);

        //open up
        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);
        sleep(200);
        if (skystoneAngle < 0) {
            globalCoordinate.stop();
            turnPID(0, Direction.CLOCKWISE, maxPwr + 0.2);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            globalCoordinate.setLastEncoderCountLeft(0);
            globalCoordinate.setLastEncoderCountLeftBack(0);
            globalCoordinate.setLastEncoderCountRight(0);
            globalCoordinate.setLastEncoderCountRightBack(0);

            globalCoordinate.start();
            globalCoordinateThread.start();
        } else {
            globalCoordinate.stop();
            turnPID(0, Direction.CLOCKWISE, maxPwr + 0.2);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            globalCoordinate.setLastEncoderCountLeft(0);
            globalCoordinate.setLastEncoderCountLeftBack(0);
            globalCoordinate.setLastEncoderCountRight(0);
            globalCoordinate.setLastEncoderCountRightBack(0);

            globalCoordinate.start();
            globalCoordinateThread.start();
        }

//        topClaw.setPosition(0.34);
//        bottomClaw.setPosition(0.617);
        deliver(skystone_pos, 0);
        double brickOffset = 24;
        skystone_pos+=brickOffset;
        goToPositionSupreme(skystone_pos, TILE_INCH +3.2, maxPwr, 0, 1);
        deliver(skystone_pos, -brickOffset +SKYSTONE_INCH*1.5);
        globalCoordinate.stop();
        turnPID(270, Direction.CLOCKWISE, 0.3);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        globalCoordinate.setLastEncoderCountLeft(0);
        globalCoordinate.setLastEncoderCountLeftBack(0);
        globalCoordinate.setLastEncoderCountRight(0);
        globalCoordinate.setLastEncoderCountRightBack(0);

        globalCoordinate.start();
        globalCoordinateThread.start();
        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + -brickOffset+SKYSTONE_INCH*1.5, 47.25-17.5+4, maxPwr, 270, 1);
        //foundation
        ls.setPosition(0.5);
        rs.setPosition(0.5);
        sleep(300);
        goToPositionSupreme(TILE_INCH*5-17.5-2, 2, maxPwr, 270, 1);
        ls.setPosition(0);
        rs.setPosition(0);
        sleep(100);
        goToPositionSupreme(TILE_INCH*2-17.5, 2, maxPwr, 270, 1);

    }
    public void deliver(double skystone_pos, double offset) throws InterruptedException{

        //open up
        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);
        goToPositionSupreme(skystone_pos, TILE_INCH + 6.2, maxPwr + 0.8, 0, 1);
        //globalCoordinate.setGlobalX(globalCoordinate.getGlobalX() + 2 * SKYSTONE_INCH);


        topClaw.setPosition(0.855);
        sleep(500);
        bottomClaw.setPosition(0.166);
        goToPositionSupreme(skystone_pos, TILE_INCH + 3.2, maxPwr + 0.8, 0, 1);


//        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 3.2, 0.9, 0, 2);

        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 9.2, maxPwr, 0, 1);
        bottomClaw.setPosition(0.500);
        sleep(100);
        topClaw.setPosition(0.2329);

        goToPositionSupreme(skystone_pos + 4 * TILE_INCH-4 + offset, TILE_INCH + 3.2, maxPwr + 0.8, 0, 1);
        bottomClaw.setPosition(0.167);
        topClaw.setPosition(0.855);
    }
}