package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.MecanumAuto;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper.SkyScraper;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Mat;


import java.net.PortUnreachableException;
import java.sql.Time;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

//@Disabled
//@Autonomous
public class StoneFinder extends opencvSkystoneDetector {

    protected BNO055IMU imu;//For detecting angles of rotation

    enum Mode {OPEN, CLOSE}
    enum Side {LEFT, RIGHT}
    enum GAGE {FORWARD, REVERSE}

    //driveTrain
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor rightIntake;
    DcMotor leftIntake;

    Servo hook;

    Servo rs;
    Servo ls;

    Servo tape;

    DcMotor rightLift;
    DcMotor leftLift;

    Servo topClaw;
    Servo bottomClaw;

    final double intakePwr = 0.5;
    final double maxLiftPwr = 0.3;
    double currentLiftPwr = 0;
    final double drivePwrMax = 0.7;

    static final double COUNTS_PER_REVOLUTION = 537.6; //20:1
    static final double DRIVE_GEAR_REDUCTION = 1; //This is < 1.0 if geared up
    static final double WHEEL_DIAMETER_INCHES = 3.93701;
    static final double WHEEL_PERIMETER_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    enum Direction {CLOCKWISE, COUNTERCLOCKWISE}


    enum SkystonePositions {LEFT, MID, RIGHT}
    //Field Variables
    final double TILE_INCH = 22.75;
    final double CONNECTION_INCH = 1.125;
    final double SKYSTONE_INCH = 8;
    final double ROBOT_LENGTH_INCH = 17.85;
    final double ROBOT_WIDTH_INCH = 17.7;


    PIDMichael PID = new PIDMichael(0.001, Math.pow(10, -12), 10000); //experimentally found

    double skystoneAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

    }

    public void initialize() throws InterruptedException {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //driveTrain
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rightIntake = hardwareMap.dcMotor.get("rightIntake");
//        leftIntake = hardwareMap.dcMotor.get("leftIntake");
//
//        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        hook = hardwareMap.servo.get("hook");
//        hook.setDirection(Servo.Direction.FORWARD);
//
        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);

//        rightLift = hardwareMap.dcMotor.get("rightLift");
//        leftLift = hardwareMap.dcMotor.get("leftLift");
//        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
//
        topClaw = hardwareMap.servo.get("top");
        bottomClaw = hardwareMap.servo.get("bot");
        topClaw.setDirection(Servo.Direction.REVERSE);
        bottomClaw.setDirection(Servo.Direction.REVERSE);

        tape = hardwareMap.servo.get("tape");
        tape.setDirection(Servo.Direction.REVERSE);
        tape.setPosition(0.5);

        topClaw.setPosition(0.4);
        bottomClaw.setPosition(0.3);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightIntake = hardwareMap.dcMotor.get("rightIntake");
//        leftIntake = hardwareMap.dcMotor.get("leftIntake");
//
//        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        claw = hardwareMap.servo.get("claw");
//        claw.setDirection(Servo.Direction.FORWARD);
//

//
//        rightLift = hardwareMap.dcMotor.get("rightLift");
//        leftLift = hardwareMap.dcMotor.get("leftLift");
//        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        //find skystone pos
        super.runOpMode(); //init camera

        double mid = 0;

        while (!isStarted()) {
            if (valLeft == 0) {
                skystoneAngle = mid + 14;
            }
            else if (valMid == 0) {
                skystoneAngle = mid;
            }
            else {
                skystoneAngle = mid - 14;
            }

            telemetry.addData("angle", skystoneAngle);
            telemetry.update();
        }

        phoneCam.closeCameraDevice();


    }

    public void telemetry() {
        telemetry.addData("fl_pwr", fl.getPower());
        telemetry.addData("fr_pwr", fr.getPower());
        telemetry.addData("bl_pwr", bl.getPower());
        telemetry.addData("br_pwr", br.getPower());
        telemetry.addData("fl_target_left", fl.getTargetPosition() - fl.getCurrentPosition());
        telemetry.addData("fr_target_left", fr.getTargetPosition() - fr.getCurrentPosition());
        telemetry.addData("bl_target_left", bl.getTargetPosition() - bl.getCurrentPosition());
        telemetry.addData("br_target_left", br.getTargetPosition() - br.getCurrentPosition());
        telemetry.update();
    }

    public void moveInches (double inches, double angle, long sleep) {
        double power = drivePwrMax;
        int target = (int) (inches * COUNTS_PER_INCH);
        runToPosition(target, angle);
        transformation(angle, power, fr, fl, br, bl);
        while (fr.isBusy() || fl.isBusy() || br.isBusy() || bl.isBusy()) {
            telemetry();
        }
        stopAndResetMotor();
        sleep(sleep);
    }
    public void moveInchesPID(double inches, double angle, long sleep) {
        double power = drivePwrMax, snipPwr, integral = 0.65; //integral is the lowest power level for the robot to move
        int target = (int) (inches * COUNTS_PER_INCH);
        runToPosition(target, angle);
        transformation(angle, power, fr, fl, br, bl);
        PID.initPID(target, System.nanoTime());
        while (fr.isBusy() || fl.isBusy() || br.isBusy() || bl.isBusy()) {
            if (Math.abs(fr.getPower()) > 0) snipPwr = PID.actuator(fr.getCurrentPosition(), System.nanoTime());
            else snipPwr = PID.actuator(fl.getCurrentPosition(), System.nanoTime());

            if (snipPwr < integral) integral = integral - snipPwr;
            power = Range.clip(snipPwr + integral, 0, 1) * drivePwrMax;
            transformation(Math.toRadians(angle), power, fr, fl, br, bl);

            telemetry();
        }
        stopAndResetMotor();
        long timeT = System.currentTimeMillis();
        sleep(sleep);
    }
    public void moveInchesGyro(double inches, double angle, long sleep, int dialation) {
        double initialAngle = getAbsoluteHeading(), currentAngle, adjust = 0; // 80, 0.3   80,0.35 0.44 67
        double power = drivePwrMax, snipPwr, integral = 0.30; //integral is the lowest power level for the robot to move
        int target = (int) (inches * COUNTS_PER_INCH);
        runToPosition(target, angle);
        //PID.initPID(target, System.nanoTime());
        angle = Math.toRadians(angle);
        while (fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
            double average = Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
            currentAngle = getAbsoluteHeading();
//            if (Math.abs(fr.getPower()) > 0) snipPwr = average / target;
            snipPwr = average / target;

            if (snipPwr < integral) integral = integral - snipPwr;
            power = Range.clip(snipPwr + integral, 0, 1);
            adjust = initialAngle - currentAngle;

            if ((adjust < 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust > 0 && (initialAngle > 0 && currentAngle < 0)) { //need clockwise
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
            } else if ((adjust > 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust < 0 && (initialAngle > 0 && currentAngle < 0)) {
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
            } else {
                fr.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
                fl.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                br.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                bl.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
            }
            telemetry();
        }
        stopAndResetMotor();
        sleep(sleep, dialation + "");
    }
    public void moveInchesGyro(double inches, double angle, long sleep) {
        double initialAngle = getAbsoluteHeading(), currentAngle, adjust = 0, dialation = 80; // 80, 0.3   80,0.35 0.44 67
        double power = drivePwrMax, snipPwr, integral = 0.3; //integral is the lowest power level for the robot to move
        int target = (int) (inches * COUNTS_PER_INCH);
        runToPosition(target, angle);
        angle = Math.toRadians(angle);
        while (fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
            double average = Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
            currentAngle = getAbsoluteHeading();
            snipPwr = average / target;

            if (snipPwr < integral) integral = integral - snipPwr;
            power = Range.clip(snipPwr + integral, 0, 1);
            adjust = initialAngle - currentAngle;

            if ((adjust < 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust > 0 && (initialAngle > 0 && currentAngle < 0)) { //need clockwise
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
            } else if ((adjust > 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust < 0 && (initialAngle > 0 && currentAngle < 0)) {
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
            } else {
                fr.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
                fl.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                br.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                bl.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
            }
            telemetry();
        }
        stopAndResetMotor();
        sleep(sleep);
    }
    public void moveInchesPIDandGyro(double inches, double angle, long sleep) {
        double initialAngle = getAbsoluteHeading(), currentAngle, adjust = 0, dialation = 75;
        double power = drivePwrMax, snipPwr, integral = 0.30; //integral is the lowest power level for the robot to move
        int target = (int) (inches * COUNTS_PER_INCH);
        runToPosition(target, angle);
        PID.initPID(target, System.nanoTime());
        angle = Math.toRadians(angle);
        while (fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
            currentAngle = getAbsoluteHeading();
            if (Math.abs(fr.getPower()) > 0) snipPwr = PID.actuator(fr.getCurrentPosition(), System.nanoTime());
            else snipPwr = PID.actuator(fl.getCurrentPosition(), System.nanoTime());

            if (snipPwr < integral) integral = integral - snipPwr;
            power = Range.clip(snipPwr + integral, 0, 1) * drivePwrMax;
            adjust = initialAngle - currentAngle;

            if ((adjust < 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust > 0 && (initialAngle > 0 && currentAngle < 0)) { //need clockwise
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
            } else if ((adjust > 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust < 0 && (initialAngle > 0 && currentAngle < 0)) {
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
            } else {
                fr.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
                fl.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                br.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                bl.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
            }
            telemetry();
        }
        stopAndResetMotor();
        sleep(sleep);
    }
    public void GyroSupreme(double inches, double angle, long sleep) {
        double initialAngle = getAbsoluteHeading(), currentAngle, adjust = 0, dialation = 80; // 80, 0.3   80,0.35 0.44 67
        double power = drivePwrMax, snipPwr, integral = 0.3; //integral is the lowest power level for the robot to move
        double circ = Math.PI * (3.93701);
        setTargetPosition(inches, angle);
        int target = (int) (Math.abs(fr.getTargetPosition()) + Math.abs(fl.getTargetPosition()) + Math.abs(br.getTargetPosition()) + Math.abs(bl.getTargetPosition())) / 4;
        runToPosition();
        angle = Math.toRadians(angle);

        while (Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4 <= target) {
            double average = Math.abs(fr.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4;
            currentAngle = getAbsoluteHeading();
            snipPwr = average / target;

            if (snipPwr < integral) integral = integral - snipPwr;
            power = Range.clip(snipPwr + integral, 0, 1);
            adjust = initialAngle - currentAngle;

            if ((adjust < 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust > 0 && (initialAngle > 0 && currentAngle < 0)) { //need clockwise
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
            } else if ((adjust > 0 && !(initialAngle > 0 && currentAngle < 0)) || adjust < 0 && (initialAngle > 0 && currentAngle < 0)) {
                fr.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                fl.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
                br.setPower(Math.abs(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power) + Math.abs(adjust / dialation));
                bl.setPower(Math.abs(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power) - Math.abs(adjust / dialation));
            } else {
                fr.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
                fl.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                br.setPower(Math.sqrt(2) * Math.cos(angle - Math.PI / 4) * power);
                bl.setPower(Math.sqrt(2) * Math.sin(angle - Math.PI / 4) * power);
            }
            telemetry();
        }
        stopAndResetMotor();
        sleep(sleep);
    }
    public void turn(double angle, Direction direction, double maxPwr) {
        runWithoutEncoders();
        flipMechanic(fr, fl, br, bl);
        double integral = 0.5;
        double error = 5;
        double angleLeft = Math.abs(getAbsoluteHeading() - angle);
        double firstDiff = Math.abs(getAbsoluteHeading() - angle);
        if (direction == Direction.CLOCKWISE) {
            while (opModeIsActive() && angleLeft > error) {
                angleLeft = Math.abs(getAbsoluteHeading() - angle);
                fr.setPower(-maxPwr);
                fl.setPower(maxPwr);
                br.setPower(-maxPwr);
                bl.setPower(maxPwr);
                telemetry.addData("fr_pwr", fr.getPower());
                telemetry.addData("fl_pwr", fl.getPower());
                telemetry.addData("imu", getAbsoluteHeading());
                telemetry.addData("angleLeft", angleLeft);
                telemetry.addData("firstDiff", firstDiff);
                telemetry.addData("clockwise", "true");
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && angleLeft > error) {
                angleLeft = Math.abs(getAbsoluteHeading() - angle);
                fr.setPower(maxPwr);
                fl.setPower(-maxPwr);
                br.setPower(maxPwr);
                bl.setPower(-maxPwr);
                telemetry.addData("fr_pwr", fr.getPower());
                telemetry.addData("fl_pwr", fl.getPower());
                telemetry.addData("imu", getAbsoluteHeading());
                telemetry.addData("angleLeft", angleLeft);
                telemetry.addData("firstDiff", firstDiff);
                telemetry.addData("clockwise", "true");
                telemetry.update();
            }
        }
        flipMechanic(fr, fl, br, bl);
        stopAndResetMotor();
    }
    public void turnPID(double angle, Direction direction, double power) {
        runWithoutEncoders();
        //flipMechanic(fr, fl, br, bl);
        double angleLeft = Math.abs(getAbsoluteHeading() - angle);
        double firstDiff = Math.abs(getAbsoluteHeading() - angle);
        double integral = 0.3;
        double error = 3;
        if (direction == Direction.CLOCKWISE) {
            while (opModeIsActive() && angleLeft > error) {
                angleLeft = Math.abs(getAbsoluteHeading() - angle);
                if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                else power = integral;
                fr.setPower(-power);
                fl.setPower(power);
                br.setPower(-power);
                bl.setPower(power);
                telemetry.addData("fr_pwr", fr.getPower());
                telemetry.addData("fl_pwr", fl.getPower());
                telemetry.addData("imu", getAbsoluteHeading());
                telemetry.addData("angleLeft", angleLeft);
                telemetry.addData("firstDiff", firstDiff);
                telemetry.addData("clockwise", "true");
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && angleLeft > error) {
                angleLeft = Math.abs(getAbsoluteHeading() - angle);
                if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                else power = integral;
                fr.setPower(power);
                fl.setPower(-power);
                br.setPower(power);
                bl.setPower(-power);
                telemetry.addData("fr_pwr", fr.getPower());
                telemetry.addData("fl_pwr", fl.getPower());
                telemetry.addData("imu", getAbsoluteHeading());
                telemetry.addData("angleLeft", angleLeft);
                telemetry.addData("firstDiff", firstDiff);
                telemetry.addData("clockwise", "true");
                telemetry.update();
            }
        }
        //flipMechanic(fr, fl, br, bl);
        stopAndResetMotor();
    }

    public void foundation(Mode position, int sleep) throws InterruptedException {
        if (position == position.OPEN) {
            ls.setPosition(0);
            rs.setPosition(0);
            sleep(sleep);
        }
        else if (position == position.CLOSE) {
            ls.setPosition(0.5);
            rs.setPosition(0.5);
            sleep(sleep);
        }
    }


    public void stopAndResetMotor() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public final void sleep(long time, String input) {
        telemetry.addData("State Finished", input);
        telemetry.update();
        super.sleep(time);
    }


    public void YLinearInchesGyro(double inches, GAGE direction, double power, double absoluteHeading) {
        //math
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * COUNTS_PER_REVOLUTION);
        //initialize
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftPower;
        double rightPower;
        double change;
        //direction
        if(direction == GAGE.FORWARD) {
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
            change = direction == GAGE.FORWARD ? getAbsoluteHeading() : -getAbsoluteHeading();
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
        stopAndResetMotor();
        reinitialize();
    }

    public void XLinearInchesGyro(double inches, Side direction, double power, double absoluteHeading) {
        //math
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * COUNTS_PER_REVOLUTION);
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
        stopAndResetMotor();
        reinitialize();
    }
    public void reinitialize() {
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void runToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runToPosition(int target, double angle) {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(double target, double angle) {
        angle = Math.toRadians(angle);
        double fltarget = target, frtarget = target;
        double flProportion = (Math.sqrt(2) * Math.cos(angle - Math.PI / 4)) / (Math.sqrt(2) * Math.sin(angle - Math.PI / 4));
        double frProportion = (Math.sqrt(2) * Math.sin(angle - Math.PI / 4)) / (Math.sqrt(2) * Math.cos(angle - Math.PI / 4));
        if (frProportion > flProportion) {
            frtarget = target;
            fltarget = target * Math.abs(flProportion);
        } else {
            fltarget = target;
            frtarget = target * Math.abs(frProportion);
        }
        if (Math.sqrt(2) * Math.cos(angle - Math.PI / 4) < 0) fltarget *= -1;
        if (Math.sqrt(2) * Math.sin(angle - Math.PI / 4) < 0) frtarget *= -1;
        fl.setTargetPosition((int) fltarget);
        fr.setTargetPosition((int) frtarget);
        bl.setTargetPosition((int) frtarget);
        br.setTargetPosition((int) fltarget);
    }

    public void runToPosition(int frtarget, int fltarget) {
        fl.setTargetPosition(fltarget);
        fr.setTargetPosition(frtarget);
        bl.setTargetPosition(frtarget);
        br.setTargetPosition(fltarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //angle in radians
    public void transformation (double angles, double power, DcMotor... motors) {
        //fr fl br bl
        angles = Math.toRadians(angles);
        for (int i = 0; i < motors.length; i++) {
            switch (i) {
                case 0 : fr.setPower(Math.sqrt(2) * Math.sin(angles - Math.PI / 4) * power); break;
                case 1 : fl.setPower(Math.sqrt(2) * Math.cos(angles - Math.PI / 4) * power); break;
                case 2 : br.setPower(Math.sqrt(2) * Math.cos(angles - Math.PI / 4) * power); break;
                case 3 : bl.setPower(Math.sqrt(2) * Math.sin(angles - Math.PI / 4) * power); break;
            }
        }

    }


    public void moveClaw(double position1, double position2) {
        topClaw.setPosition(position1);
        bottomClaw.setPosition(position2);
    }

    public void flipMechanic(DcMotor... motors) {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i].getDirection() == DcMotorSimple.Direction.FORWARD) motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            else motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    //    public void moveForwardInches(double inches, long sleep) throws InterruptedException {
//        double power = drivePwrMax;
//        int target = (int) (inches * COUNTS_PER_INCH);
//
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        fl.setTargetPosition(target);
//        fr.setTargetPosition(target);
//        bl.setTargetPosition(target);
//        br.setTargetPosition(target);
//
//        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (inches < 0) {
//            fl.setPower(-power);
//            fr.setPower(-power);
//            bl.setPower(-power);
//            br.setPower(-power);
//        } else {
//            fl.setPower(power);
//            fr.setPower(power);
//            bl.setPower(power);
//            br.setPower(power);
//        }
//
//        while(opModeIsActive() &&
//                (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {
//            telemetry();
//        }
//
//        stopMotor();
//        sleep(sleep, "moveForwardInches " + inches + " inches");
//    }

//    public void moveSidewaysInches(double inches, long sleep) throws InterruptedException{
//        double power = drivePwrMax;
//        int target = (int) (inches * COUNTS_PER_INCH);
//
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        fl.setTargetPosition(target);
//        fr.setTargetPosition(-target);
//        bl.setTargetPosition(-target);
//        br.setTargetPosition(target);
//
//        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (inches < 0) {
//            fl.setPower(-power);
//            fr.setPower(power);
//            bl.setPower(power);
//            br.setPower(-power);
//        }
//        else {
//            fl.setPower(power);
//            fr.setPower(-power);
//            bl.setPower(-power);
//            br.setPower(power);
//        }
//
//        while(opModeIsActive() &&
//                (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {
//        telemetry();
//        }
//
//        stopMotor();
//        sleep(sleep, "moveSidewaysInches " + inches + " inches");
//    }

//    public void moveDiagonally(double inches, double angle, long sleep) throws InterruptedException {
//        double power = drivePwrMax;
//        int target = (int) (inches * COUNTS_PER_INCH);
//
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        fl.setPower()
//
//        while(opModeIsActive() &&
//                (? fl.isBusy() : fr.isBusy()) {
//
//            if (inches < 0) {
//                fl.setPower(-power);
//                fr.setPower(power);
//                bl.setPower(power);
//                br.setPower(-power);
//            }
//            else {
//                fl.setPower(power);
//                fr.setPower(-power);
//                bl.setPower(-power);
//                br.setPower(power);
//            }
//
//            telemetry.addData("fl_pwr", fl.getPower());
//            telemetry.addData("fr_pwr", fr.getPower());
//            telemetry.addData("bl_pwr", bl.getPower());
//            telemetry.addData("br_pwr", br.getPower());
//            telemetry.addData("fl_target_left", target - fl.getCurrentPosition());
//            telemetry.update();
//        }
//        stopMotor();
//        sleep(sleep, "moveDiagonally " + inches + " inches");
//    }
}


