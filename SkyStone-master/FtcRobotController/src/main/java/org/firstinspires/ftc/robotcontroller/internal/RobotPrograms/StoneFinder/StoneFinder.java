package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.net.PortUnreachableException;
import java.util.ArrayList;

//@Disabled
@Autonomous
public class StoneFinder extends opencvSkystoneDetector {

    protected BNO055IMU imu; //For detecting angles of rotation

    //driveTrain
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor rightIntake;
    DcMotor leftIntake;

    Servo claw;

    Servo rs;
    Servo ls;

    DcMotor rightLift;
    DcMotor leftLift;

    final double intakePwr = 0.5;
    final double maxLiftPwr = 0.3;
    double currentLiftPwr = 0;
    double drivePwrMax = 0.8;

    static final double COUNTS_PER_REVOLUTION = 537.6; //20:1
    static final double DRIVE_GEAR_REDUCTION = 1; //This is < 1.0 if geared up
    static final double WHEEL_DIAMETER_INCHES = 3.93701;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    enum Direction {CLOCKWISE, COUNTERCLOCKWISE}

    PIDMichael PID = new PIDMichael(0.001, Math.pow(10, -12), 10000); //experimentally found

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

        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");

        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);

        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);

        rightLift = hardwareMap.dcMotor.get("rightLift");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
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
        transformation(Math.toRadians(angle), power, fr, fl, br, bl);
        while (fr.isBusy() || fl.isBusy() || br.isBusy() || bl.isBusy()) {}
        stopAndResetMotor();
        sleep(sleep);
    }
    public void moveInchesPID(double inches, double angle, long sleep) {
        double power = drivePwrMax, snipPwr, integral = 0.5; //integral is the lowest power level for the robot to move
        int target = (int) (inches * COUNTS_PER_INCH);
        runToPosition(target, angle);
        transformation(Math.toRadians(angle), power, fr, fl, br, bl);
        PID.initPID(target, System.nanoTime());
        while (fr.isBusy() || fl.isBusy() || br.isBusy() || bl.isBusy()) {
            if (Math.abs(fr.getPower()) > 0) snipPwr = PID.actuator(fr.getCurrentPosition(), System.nanoTime());
            else snipPwr = PID.actuator(fl.getCurrentPosition(), System.nanoTime());

            if (snipPwr < integral) integral = integral - snipPwr;
            power = Range.clip(snipPwr + integral, 0, 1) * drivePwrMax;
            transformation(Math.toRadians(angle), power, fr, fl, br, bl);

            telemetry.addData("fr_pwr", fr.getPower());
            telemetry.addData("right target_left", target - fr.getCurrentPosition());
            telemetry.addData("fl_pwr", fl.getPower());
            telemetry.addData("left target_left", target - fl.getCurrentPosition());
//            telemetry.addData("p", pid.p * pid.error);
//            telemetry.addData("i", pid.i * pid.integral);
//            telemetry.addData("d", pid.d * pid.differential);

            telemetry.update();
        }
        stopAndResetMotor();
        sleep(sleep);
    }
    public void moveInchesGyro() {}
    public void moveInchesPIDandGyro() {}
    public void turn() {
        runWithoutEncoders();

    }
    public void turnPID() {}


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

    public void runToPosition(int target, double angle) {
        double fltarget = target, frtarget = target;
        if (Math.sqrt(2) * Math.sin(angle - Math.PI / 4) > Math.sqrt(2) * Math.cos(angle - Math.PI / 4)) {
            frtarget = target;
            fltarget = target * (Math.sqrt(2) * Math.cos(angle - Math.PI / 4)) / (Math.sqrt(2) * Math.sin(angle - Math.PI / 4));
        } else {
            fltarget = target;
            frtarget = target * (Math.sqrt(2) * Math.sin(angle - Math.PI / 4)) / (Math.sqrt(2) * Math.cos(angle - Math.PI / 4));
        }

        fl.setTargetPosition((int) fltarget);
        fr.setTargetPosition((int) frtarget);
        bl.setTargetPosition((int) frtarget);
        br.setTargetPosition((int) fltarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        for (int i = 0; i < motors.length; i++) {
            switch (motors[i].toString()) {
                case "fr": fr.setPower(Math.sqrt(2) * Math.sin(angles - Math.PI / 4) * power); break;
                case "bl": bl.setPower(Math.sqrt(2) * Math.sin(angles - Math.PI / 4) * power); break;
                case "fl": fl.setPower(Math.sqrt(2) * Math.cos(angles - Math.PI / 4) * power); break;
                case "br": br.setPower(Math.sqrt(2) * Math.cos(angles - Math.PI / 4) * power); break;
            }
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


