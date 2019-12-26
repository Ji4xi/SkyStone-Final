package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;

import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@Autonomous
public class RangerAuto extends LinearOpMode {
    enum Direction {
        CLOCKWISE, COUNTERCLOCKWISE
    }
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    final double DRIVE_PWR = 0.8;
    final double TURN_PWR = 0.1;

    double error = 5;

    static final double COUNTS_PER_REVOLUTION = 1120; //40:1

    static final double DRIVE_GEAR_REDUCTION = 1; //This is < 1.0 if geared up
    static final double WHEEL_DIAMETER_INCHES = 3;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    PIDMichael pid = new PIDMichael(0.001, Math.pow(10, -12), 10000); //experimentally found

    boolean mode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //0.1 for turning, 0.8 for moving forward
        // don't get too close to 180 and -180
        initialize();
        waitForStart();

        mode = true;
        turnAbsolute(-90, Direction.CLOCKWISE, TURN_PWR);
        sleep(1000);

    }

    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //driveTrain
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");

        lm.setDirection(DcMotorSimple.Direction.FORWARD);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);

        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnAbsolute(double angle, Direction direction, double power) {
        double integral = 0.2;
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (mode) {
            double angleLeft = Math.abs(getAbsoluteHeading() - angle);
            double firstDiff = Math.abs(getAbsoluteHeading() - angle);
            if (direction == Direction.CLOCKWISE) {
                while (angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(power);
                    lm.setPower(-power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            } else {
                while (angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(-power);
                    lm.setPower(power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            }
        } else {
            angle = -angle;
            double angleLeft = Math.abs(getAbsoluteHeading() - angle);
            double firstDiff = Math.abs(getAbsoluteHeading() - angle);
            if (direction == Direction.COUNTERCLOCKWISE) {
                while (angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(power);
                    lm.setPower(-power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            } else {
                while (angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(-power);
                    lm.setPower(power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            }
        }
    }
    public void moveForward(double rev) throws InterruptedException {
        double power;
        int target = (int) (rev * COUNTS_PER_REVOLUTION);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);


        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pid.initPID(target, System.nanoTime());
        while(rm.isBusy() && lm.isBusy()) {
            power = pid.actuator(rm.getCurrentPosition(), System.nanoTime()) * DRIVE_PWR;
            if (rev < 0) {
                rm.setPower(-power);
                lm.setPower(-power);
            } else {
                rm.setPower(power);
                lm.setPower(power);
            }

            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("target_left", target - rm.getCurrentPosition());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.update();
        }
        stopMotor();
    }

    public void moveForwardInches(double inches) throws InterruptedException {
        double power;
        int target = (int) (inches * COUNTS_PER_INCH);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);

        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pid.initPID(target, System.nanoTime());

        while(opModeIsActive() && rm.isBusy() && lm.isBusy()) {
            power = pid.actuator(rm.getCurrentPosition(), System.nanoTime()) * DRIVE_PWR;
            if (inches < 0) {
                rm.setPower(-power);
                lm.setPower(-power);
            } else {
                rm.setPower(power);
                lm.setPower(power);
            }

            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("target_left", target - rm.getCurrentPosition());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.update();
        }
        stopMotor();
    }
    public void moveForwardInchesGyro(double inches) throws InterruptedException {
        double power;
        double initialAngle = getAbsoluteHeading();
        int target = (int) (inches * COUNTS_PER_INCH);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);

        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pid.initPID(target, System.nanoTime());

        while(opModeIsActive() && rm.isBusy() && lm.isBusy()) {
            power = pid.actuator(rm.getCurrentPosition(), System.nanoTime()) * DRIVE_PWR;
            double tuneConst = 100;
            if (inches < 0) {
                if (initialAngle - getAbsoluteHeading() > 0) {
                    rm.setPower(-power - (initialAngle - getAbsoluteHeading() / tuneConst));
                    lm.setPower(-power + (initialAngle - getAbsoluteHeading() / tuneConst));
                } else {
                    rm.setPower(-power + (initialAngle - getAbsoluteHeading() / tuneConst));
                    lm.setPower(-power - (initialAngle - getAbsoluteHeading() / tuneConst));
                }
            } else {
                if (initialAngle - getAbsoluteHeading() > 0) {
                    rm.setPower(power + (initialAngle - getAbsoluteHeading() / tuneConst));
                    lm.setPower(power - (initialAngle - getAbsoluteHeading() / tuneConst));
                } else {
                    rm.setPower(power - (initialAngle - getAbsoluteHeading() / tuneConst));
                    lm.setPower(power + (initialAngle - getAbsoluteHeading() / tuneConst));
                }
            }

            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("target_left", target - rm.getCurrentPosition());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.update();
        }
        stopMotor();
    }

    public void stopMotor() {
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public final void sleep(long time, String input) {
        telemetry.addData("State Finished", input);
        telemetry.update();
        super.sleep(time);
    }
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}

