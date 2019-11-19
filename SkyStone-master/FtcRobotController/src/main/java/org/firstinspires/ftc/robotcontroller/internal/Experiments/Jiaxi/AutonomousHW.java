package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@Autonomous (name = "JiaxiAutonomous")
public class AutonomousHW extends LinearOpMode {
    protected BNO055IMU imu; //DO NOT UNDERSTAND HOW TO PROGRAM SENSOR
    DcMotor lm;
    DcMotor rm;
    double error = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        final double COUNTS_PER_REVOLUTION = 1120;
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        initializeIMU();
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setTargetPosition((int) (2 * COUNTS_PER_REVOLUTION));
        rm.setTargetPosition((int) (2 * COUNTS_PER_REVOLUTION));
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        lm.setPower(0.5);
        rm.setPower(0.5);
        while(lm.isBusy()) {
        }
        lm.setPower(0);
        rm.setPower(0);
        turn(90, true, 0.2);
        while(lm.isBusy()){
        }
        lm.setPower(0);
        rm.setPower(0);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setTargetPosition((int) (3 * COUNTS_PER_REVOLUTION));
        rm.setTargetPosition((int) (3 * COUNTS_PER_REVOLUTION));
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setPower(0.5);
        lm.setPower(0.5);
        while(lm.isBusy() && rm.isBusy()) {
        }
        lm.setPower(0);
        rm.setPower(0);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void initializeIMU() { //DO NOT KNOW HOW TO PROGRAM THIS
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }
    public double getAbsoluteHeading() { //NO IDEA WHAT TO DO HERE
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void turn(double angles, boolean clockwise, double power) {
        angles = Math.abs(angles);
        if (Math.abs(getAbsoluteHeading() - angles) > error) {
            if (clockwise) {
                rm.setDirection(DcMotorSimple.Direction.FORWARD);
                lm.setDirection(DcMotorSimple.Direction.FORWARD);
                rm.setPower(power);
                lm.setPower(power);
            }
            if (!clockwise) {
                rm.setDirection(DcMotorSimple.Direction.REVERSE);
                lm.setDirection(DcMotorSimple.Direction.REVERSE);
                rm.setPower(power);
                lm.setPower(power);
            }
            turn(angles, clockwise, power);
        }
        else {
            lm.setPower(0);
            rm.setPower(0);
        }
    }
}
