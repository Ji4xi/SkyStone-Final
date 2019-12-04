package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Turn extends LinearOpMode {
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    //final double COUNTS_PER_REVOLUTION = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeIMU();
        waitForStart();
        turnAbsolute(90, true, 0.5);
    }

    public void initializeIMU(){
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        rm = hardwareMap.dcMotor.get("rm");
        lm = hardwareMap.dcMotor.get("lm");
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void turnAbsolute( double degrees, boolean clockwise, double power){
        if (clockwise) {
//            if (getAbsoluteHeading() > -(degrees)) {
//                rm.setPower(-(power));
//                lm.setPower(power);
//            }
            rm.setPower(-(power));
            lm.setPower(power);
            while (getAbsoluteHeading() > -(degrees)) {
            }
            rm.setPower(0);
            lm.setPower(0);

        }
        else if (!clockwise) {
//            if (getAbsoluteHeading() < degrees) {
//                rm.setPower(power);
//                lm.setPower(-(power));
//            }
            rm.setPower(power);
            lm.setPower(-(power));
            while (getAbsoluteHeading() < degrees) {
            }
            rm.setPower(0);
            lm.setPower(0);
        }
        turnAbsolute(degrees, clockwise, power);
    }
}
