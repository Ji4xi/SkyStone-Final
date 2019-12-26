package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@Autonomous
public class Gyro extends LinearOpMode {
    enum Direction {
        FORWARD, REVERSE
    }
    protected BNO055IMU imu; //For detecting angles of rotation

    //drive train
    DcMotor r, l;
    final double tpr = 1120;
    double correction;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        LinearInchesGyro(60, Direction.FORWARD, 0.5, 0);
        sleepSeconds(2);
        LinearInchesGyro(60, Direction.REVERSE, 0.3, 0);
    }

    public void initialize() throws InterruptedException {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //drive train
        r = hardwareMap.dcMotor.get("r");
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l = hardwareMap.dcMotor.get("l");
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //angle

    }

    public void LinearInchesGyro(double inches, Direction direction, double power, double absoluteHeading) {
        //math
        double circ = Math.PI * (3);
        int target = (int) (inches / circ * tpr);
        //initialize
        r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftPower;
        double rightPower;
        double change = getAbsoluteHeading();
        //direction
        if (direction == Direction.FORWARD) {
            r.setDirection(DcMotorSimple.Direction.FORWARD);
            l.setDirection(DcMotorSimple.Direction.REVERSE);
            change = getAbsoluteHeading();
        }
        if (direction == Direction.REVERSE) {
            r.setDirection(DcMotorSimple.Direction.REVERSE);
            l.setDirection(DcMotorSimple.Direction.FORWARD);
            change = -getAbsoluteHeading();
        }
        //run program
        r.setTargetPosition(target);
        l.setTargetPosition(target);
        r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        l.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (r.isBusy() && l.isBusy()) {
            //reset power
            leftPower = power + (getAbsoluteHeading() - absoluteHeading)/100;
            rightPower = power - (getAbsoluteHeading() - absoluteHeading)/100;
            //bounds
            leftPower = Range.clip(leftPower, -power, power);
            rightPower = Range.clip(rightPower, -power, power);
            r.setPower(rightPower);
            l.setPower(leftPower);

            //telemetry
            telemetry.addData("absoluteHeading", getAbsoluteHeading());
            telemetry.addData("leftPower", l.getPower());
            telemetry.addData("rightPower", r.getPower());
            telemetry.update();

        }
        motorStop();
        reinitialize();
    }

    public void motorStop() {
        //set powers
        r.setPower(0);
        l.setPower(0);
        //reset
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void reinitialize() {
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        l.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void sleepSeconds(double seconds) throws InterruptedException {
        int conversion = (int) (seconds * 1000);
        sleep(conversion);
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}

