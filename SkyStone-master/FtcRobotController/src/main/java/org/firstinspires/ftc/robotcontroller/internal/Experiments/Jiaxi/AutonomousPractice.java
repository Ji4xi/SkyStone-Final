package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous
public class AutonomousPractice extends LinearOpMode {
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }

    public void cpr (int motorType) {
        double x = 0;
        if (motorType == 40) {
            x = 1120;
        }
        else if (motorType == 20) {
            x = 537.6;
        }
        else if (motorType == 60) {
            x = 1680;
        }
        double COUNTS_PER_REVOLUTION = x;
    }
}
