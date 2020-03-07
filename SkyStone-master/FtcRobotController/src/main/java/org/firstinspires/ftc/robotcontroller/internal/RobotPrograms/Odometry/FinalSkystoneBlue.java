package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

@Autonomous
public class FinalSkystoneBlue extends NewStoneFinder {
    int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        maxPwr = 0.75; //0.75
        super.runOpMode();

        //bot 0.39 level to grab, 0.75 vertical up
        //top
        double skystone_pos;
        if (skystoneAngle < 0) {
            skystone_pos = -(SKYSTONE_INCH * 0 + 10);
        } else if (skystoneAngle == 0) {
            skystone_pos = -(SKYSTONE_INCH * 1 + 10);
        } else {
            skystone_pos = -(SKYSTONE_INCH * 2 + 11);
        }

        goToPositionSupreme(skystone_pos, TILE_INCH + 2.7, maxPwr, 90, 1, 0.00106, 0, 0);

        if (skystoneAngle < 0) {
            turnPID(0, Direction.CLOCKWISE, maxPwr, 0.0180, 0.01, 0.001);
        } else {
            turnPID(0, Direction.CLOCKWISE, maxPwr, 0.0180, 0.01, 0.001);
        }


        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);

        goToPositionSupreme(skystone_pos, TILE_INCH + 6.2, maxPwr, 0, 1,0.00253, 0.0, 0);
        topClaw.setPosition(0.855);
        sleep(300);
        bottomClaw.setPosition(0.166);
        goToPositionSupreme(skystone_pos, TILE_INCH - 0.5, maxPwr, 0, 1,0.002, 0.0, 0);
        if (skystoneAngle == 0) {
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8, TILE_INCH + 1.5, 0.9, 0, 1.55,0.00151, 0, 0.00034); //d: 0.00033
        } else if (skystone_pos > 0) {
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8, TILE_INCH + 1.5, 0.9, 0, 1.55,0.00153, 0, 0.00034);
        } else {
            goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8, TILE_INCH + 1.5, 0.9, 0, 1.55,0.00151, 0, 0.00034); //d: 0.00033
        }

        bottomClaw.setPosition(0.500);
        goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8, TILE_INCH + 9.3, maxPwr, 0, 1.3,0.0022, 0.0, 0.0001); //d: 0.00033
        sleep(150);
        topClaw.setPosition(0.2329);
        goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10) + 4 * TILE_INCH-7 + 25.5 - 8, TILE_INCH + 5, maxPwr, 0, 1.25,0.0023, 0.0, 0);
//
//        goToPositionHard(skystone_pos + 4 * TILE_INCH-4, TILE_INCH + 8, maxPwr-0.6, 0, 1);
        bottomClaw.setPosition(0.21);
        topClaw.setPosition(0.47);

//
        skystone_pos+=24;
        goToPositionSupreme(skystone_pos - 1 + 2, TILE_INCH + 1.5, 0.9, 0,1.5,0.00161, 0, 0.00039); //d:0.000155
        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);
        goToPositionSupreme(skystone_pos - 1 + 2 - 2, TILE_INCH + 7.4, maxPwr, 0, 1.2,0.0018, 0.0, 0);
        topClaw.setPosition(0.855);
        sleep(300);
        bottomClaw.setPosition(0.166);
        goToPositionSupreme(skystone_pos - 1 + 2 - 1, TILE_INCH - 0.5, maxPwr, 0, 1.35,0.00164, 0.0, 0); //0.0017
        goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8 + 10, TILE_INCH + 5.5, 0.9, 0, 1.7,0.001358, 0, 0.00025);
        bottomClaw.setPosition(0.500);
        goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8 + 10, TILE_INCH + 11.5, maxPwr, 0, 1.3,0.0021, 0.0, 0);//0.0020
        sleep(150);
        topClaw.setPosition(0.2329);

        turnPID(270, Direction.CLOCKWISE, maxPwr, 0.019, 0.01, 0.001);
        topClaw.setPosition(0.16);
        bottomClaw.setPosition(0.617);
        goToPositionSupreme(-(SKYSTONE_INCH * 2 + 10)+ 4 * TILE_INCH-7 + 25.5 - 8 + 10, 47.25-17.5+10.8, maxPwr, 270, 1.7,0.00195,0,0.0001);
        //foundation
        ls.setPosition(0.5);
        rs.setPosition(0.5);
        sleep(300);
        goToPositionSupreme(TILE_INCH*5-17.5-2, 1.2, 0.9, 270, 2.3,0.0015, 0, 0, 1.4);
        ls.setPosition(0);
        rs.setPosition(0);
        goToPositionSupreme(TILE_INCH*2-23.5, -0.5, 1, 270, 1.5);

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

    public double reposX(double originalX) {
        return - (originalX - 5.7);
    }

    @Override
    public void goToPositionSupreme(double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double p, double i , double d, double gyroPwr) {
        targetXPosition = reposX(targetXPosition);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double circ = Math.PI * (3.93701);
        targetXPosition = mode ? targetXPosition : -targetXPosition;

        double originalP = pd.getP();
        double originalI = pd.getI();
        double originalD = pd.getD();

        pd.setPID(p, i, d);
        double angle;
        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
        double distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
        double snipPwr = robotPower;
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double target = distance;

        //for telemetry purpose
        double initialXPos = globalCoordinate.getGlobalX();
        double initialYPos = globalCoordinate.getGlobalY();
        double XOvershoot = 0;
        double YOvershoot = 0;

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pd.initPD(target);

        while (opModeIsActive() && distance > allowableDistanceError * COUNTS_PER_INCH) {
            double globalX = globalCoordinate.getGlobalX();
            double globalY = globalCoordinate.getGlobalY();
            double normalHeading = getNormalizedHeading();
            distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalX;
            distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalY;
            angle = Math.toDegrees(Math.atan2(distanceToYTarget, distanceToXTarget));
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            snipPwr = Math.abs(pd.actuator(target - distance)) * robotPower;

            fr.setPower(transformation((angle) + 90 - normalHeading, "fr") * snipPwr + gyro(gyroAngle, "fr") * (gyroPwr));
            fl.setPower(transformation((angle) + 90 - normalHeading, "fl") * snipPwr + gyro(gyroAngle, "fl") * (gyroPwr));
            br.setPower(transformation((angle) + 90 - normalHeading, "br") * snipPwr + gyro(gyroAngle, "br") * (gyroPwr));
            bl.setPower(transformation((angle) + 90 - normalHeading, "bl") * snipPwr + gyro(gyroAngle, "bl") * (gyroPwr));

            //overshoot
            if (globalY - targetYPosition < 0 && targetYPosition - initialYPos < 0) YOvershoot = Math.abs(globalY - targetYPosition) > YOvershoot ? Math.abs(globalY - targetYPosition) : YOvershoot;
            else if (globalY - targetYPosition > 0 && targetYPosition - initialYPos > 0) YOvershoot = Math.abs(globalY - targetYPosition) > YOvershoot ? Math.abs(globalY - targetYPosition) : YOvershoot;

            if (globalX - targetXPosition < 0 && targetXPosition - initialXPos < 0) XOvershoot = Math.abs(globalX - targetXPosition) > XOvershoot ? Math.abs(globalX - targetXPosition) : XOvershoot;
            else if (globalX - targetXPosition > 0 && targetXPosition - initialXPos > 0) XOvershoot = Math.abs(globalX - targetXPosition) > XOvershoot ? Math.abs(globalX - targetXPosition) : XOvershoot;

            Object[] names = {"fr", "fl", "br", "bl","p_contr", "i_contr", "d_contr", "dis_left_x", "dis_left_y"};
            Object[] values = {fr.getCurrentPosition(), fl.getCurrentPosition(), br.getCurrentPosition(), bl.getCurrentPosition(),pd.getPContrb(), pd.getIContrb(), pd.getDContrb(), distanceToXTarget / COUNTS_PER_INCH, distanceToYTarget / COUNTS_PER_INCH};

            telemetry(names, values);
        }
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        pd.setPID(originalP, originalI, originalD);
    }
}
