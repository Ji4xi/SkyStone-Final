package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous
public class MecanumAuto extends LinearOpMode {
    enum Direction {FORWARD, REVERSE}
    enum Side {LEFT, RIGHT}
    enum Slope {NEGATIVE, POSITIVE}
    enum Mode {OPEN, CLOSE}
    //    enum TurnDirection {CW, CCW}
    protected BNO055IMU imu; //For detecting angles of rotation

//    PIDMichael pid = new PIDMichael(0.001, Math.pow(10, -12), 10000); //experimentally found

    //drive train
    DcMotor fr, fl, br, bl;
    final double tpr = 537.6;
    Servo ls, rs;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        foundation(Mode.OPEN);
        XLinearInchesGyro(8, Side.LEFT, 0.6, 0);
        sleepSeconds(2);
        YLinearInchesGyro(28,Direction.REVERSE, 0.8, 0);
        sleepSeconds(2);
        foundation(Mode.CLOSE);
        sleepSeconds(1);
        YLinearInchesGyro(28, Direction.FORWARD, 0.4, 0);
        sleepSeconds(1);
        foundation(Mode.OPEN);
        sleepSeconds(2);
        XLinearInchesGyro(43, Side.RIGHT, 0.6, 0);
    }

    public void initialize() throws InterruptedException {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //drive train
        fr = hardwareMap.dcMotor.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br = hardwareMap.dcMotor.get("br");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl = hardwareMap.dcMotor.get("fl");
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl = hardwareMap.dcMotor.get("bl");
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //foundation servo
        rs = hardwareMap.servo.get("rs");
        rs.setDirection(Servo.Direction.REVERSE);
        ls = hardwareMap.servo.get("ls");
        ls.setDirection(Servo.Direction.FORWARD);
    }

    public void YLinearInchesGyro(double inches, Direction direction, double power, double absoluteHeading) {
        //math
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * tpr);
        //initialize
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftPower;
        double rightPower;
        double change;
        //direction
        if (direction == Direction.FORWARD) {
            fr.setDirection(DcMotorSimple.Direction.REVERSE);
            br.setDirection(DcMotorSimple.Direction.REVERSE);
            fl.setDirection(DcMotorSimple.Direction.FORWARD);
            bl.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setDirection(DcMotorSimple.Direction.FORWARD);
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        //run program
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4 <= target) {
            //checks for direction
            change = direction == Direction.FORWARD ? getAbsoluteHeading() : -getAbsoluteHeading();
            double average = Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
            //reset power
            leftPower = power + (change - absoluteHeading)/100;
            rightPower = power - (change - absoluteHeading)/100;
            //bounds
            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);
            if (average < target / 2) {
                fr.setPower((rightPower * (average / target)) + .2);
                fl.setPower((leftPower * (average / target)) + .2);
                br.setPower((rightPower * (average / target)) + .2);
                bl.setPower((leftPower * (average / target)) + .2);
            }
            else if (average > target / 2) {
                double justSomeMath = target - average;
                //set power
                fr.setPower((rightPower * (justSomeMath / target)) + .2);
                fl.setPower((leftPower * (justSomeMath / target)) + .2);
                br.setPower((rightPower * (justSomeMath / target)) + .2);
                bl.setPower((leftPower * (justSomeMath / target)) + .2);
            }

            //telemetry
            telemetry();
        }
        motorStop();
        reinitialize();
    }
//    public void XLinearInchesGyroPID(double inches, Side direction, double power) throws InterruptedException {
//
//        //math
//        double circ = Math.PI * (3.93701);
//        int target = (int) (inches / circ * tpr);
//        ;
//        //initialize
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //direction
//        if (direction == Side.RIGHT) {
//            fr.setDirection(DcMotorSimple.Direction.FORWARD);
//            br.setDirection(DcMotorSimple.Direction.FORWARD);
//            fl.setDirection(DcMotorSimple.Direction.REVERSE);
//            bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//        else {
//            fr.setDirection(DcMotorSimple.Direction.FORWARD);
//            br.setDirection(DcMotorSimple.Direction.FORWARD);
//            fl.setDirection(DcMotorSimple.Direction.REVERSE);
//            bl.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//        fl.setTargetPosition(target);
//        bl.setTargetPosition(target);
//        fr.setTargetPosition(target);
//        br.setTargetPosition(target);
//        //run program
//        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        pid.initPID(target, System.nanoTime());
//        while (fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
//            double newpower = pid.actuator(fr.getCurrentPosition(), System.nanoTime()) * power;
//             fr.setPower(newpower);
//             fl.setPower(newpower);
//             br.setPower(newpower);
//             bl.setPower(newpower);
//            //telemetry
//            telemetry();
//        }
//        motorStop();
//        reinitialize();
//    }

    public void XLinearInchesGyro(double inches, Side direction, double power, double absoluteHeading) {
        //math
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * tpr);
        double change;
        //initialize
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double forwardPower;
        double backwardPower;
        //direction
        if (direction == Side.RIGHT) {
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setDirection(DcMotorSimple.Direction.REVERSE);
            fl.setDirection(DcMotorSimple.Direction.FORWARD);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            fr.setDirection(DcMotorSimple.Direction.REVERSE);
            br.setDirection(DcMotorSimple.Direction.FORWARD);
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        //run program
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4 <= target) {
            double average = Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
            //reset power
            forwardPower = power + (getAbsoluteHeading() - absoluteHeading)/100;
            backwardPower = power - (getAbsoluteHeading() - absoluteHeading)/100;
            //bounds
            forwardPower= Range.clip(forwardPower, -1, 1);
            backwardPower = Range.clip(backwardPower, -1, 1);

            if (average < target / 2) {
                fr.setPower((backwardPower * (average / target)) + .2);
                fl.setPower((forwardPower * (average / target)) + .2);
                br.setPower((forwardPower * (average / target)) + .2);
                bl.setPower((backwardPower * (average / target)) + .2);
            }

            else if (average > target / 2) {
                double justSomeMath = target - average;
                //set power
                fr.setPower((backwardPower * (justSomeMath / target)) + .2);
                fl.setPower((forwardPower * (justSomeMath / target)) + .2);
                br.setPower((forwardPower * (justSomeMath / target)) + .2);
                bl.setPower((backwardPower * (justSomeMath / target)) + .2);
            }

            //telemetry
            telemetry();
        }
        motorStop();
        reinitialize();
    }

    public void DiagonalInchesGyro(double inches, Slope directionA, Direction directionB, double power, double absoluteHeading) {
        //math
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * tpr);
        //initialize
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftPower;
        double rightPower;
        double change;
        //direction
        if (directionB == Direction.FORWARD) {
            br.setDirection(DcMotorSimple.Direction.REVERSE);
            fl.setDirection(DcMotorSimple.Direction.FORWARD);
            bl.setDirection(DcMotorSimple.Direction.FORWARD);
            fr.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (directionB == Direction.REVERSE) {
            br.setDirection(DcMotorSimple.Direction.FORWARD);
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        //run program
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4 <= target) {
            change = directionB == Direction.FORWARD ? getAbsoluteHeading() : -getAbsoluteHeading();
            //checks for direction
            double average = Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
            //reset power
            leftPower = power + (change - absoluteHeading)/100;
            rightPower = power - (change - absoluteHeading)/100;
            //bounds
            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);
            if (average < target / 2) {
                if (directionA == Slope.POSITIVE) {
                    fr.setPower(0);
                    fl.setPower((leftPower * (average / target)) + .2);
                    br.setPower((rightPower * (average / target)) + .2);
                    bl.setPower(0);
                }
                else {
                    fr.setPower((rightPower * (average / target)) + .2);
                    fl.setPower(0);
                    br.setPower(0);
                    bl.setPower((leftPower * (average / target)) + .2);
                }

            }
            else if (average > target / 2) {
                double justSomeMath = target - average;
                //set power
                if (directionA == Slope.POSITIVE) {
                    fr.setPower(0);
                    fl.setPower((leftPower * (justSomeMath / target)) + .2);
                    br.setPower((rightPower * (justSomeMath / target)) + .2);
                    bl.setPower(0);
                }
                else {
                    fr.setPower((rightPower * (justSomeMath / target)) + .2);
                    fl.setPower(0);
                    br.setPower(0);
                    bl.setPower((leftPower * (justSomeMath / target)) + .2);
                }
            }

            //telemetry
            telemetry();
        }
        motorStop();
        reinitialize();
    }
    public void foundation(Mode mode) {
        if (mode == Mode.OPEN) {
            ls.setPosition(0);
            rs.setPosition(0);
        }
        else if (mode == Mode.CLOSE) {
            ls.setPosition(0.5);
            rs.setPosition(0.5);
        }
    }

    public void motorStop() {
        //set powers
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        fl.setPower(0);
        //reset
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void reinitialize() {
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void sleepSeconds(double seconds) throws InterruptedException {
        int conversion = (int) (seconds * 1000);
        sleep(conversion);
    }

    public void telemetry(){
        telemetry.addData("fr revolutions", fr.getCurrentPosition());
        telemetry.addData("fl revolutions", fl.getCurrentPosition());
        telemetry.addData("br revolutions", br.getCurrentPosition());
        telemetry.addData("bl revolutions", bl.getCurrentPosition());
        telemetry.addData("fr power", fr.getPower());
        telemetry.addData("fl power", fl.getPower());
        telemetry.addData("bl power", bl.getPower());
        telemetry.addData("br power", br.getPower());
        telemetry.addData("absolute heading", getAbsoluteHeading());
        telemetry.update();
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
