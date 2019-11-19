package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class ActualAutonomousTestKevin extends LinearOpMode {
    protected BNO055IMU imu;
    double error = 5;
    final int COUNTS_PER_REV = 1120;
    DcMotor rm;
    DcMotor lm;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //phase 1
        moveForward(0.8, 5);
        sleep(2000);
        //phase 2
        turnAbsolute(0.3, -90);
        sleep(1000);
        //phase 3
        moveForward(0.8, 5);
        sleep(2000);
        //phase 4
        turnAbsolute(0.3, -90);
        sleep(1000);
        //phase 5
        moveForward(0.8, 5);
        sleep(2000);
        //phase 6
        turnAbsolute(0.3, -90);
        sleep(1000);
        //phase 7
        moveForward(0.8, 5);
        sleep(2000);
        //phase 8
        turnAbsolute(0.3, -90);
        sleep(1000);

    }
    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnAbsolute(double power, double angle){
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(getAbsoluteHeading() - angle) > error) {
            if (angle > 0) {
                rm.setPower(power);
                lm.setPower(-power);
            } else {
                rm.setPower(-power);
                lm.setPower(power);
            }
            turnAbsolute(power, angle);
        } else {
            stopMotor();
        }
    }
    public void moveForward(double power, double rev) {
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition((int) (rev * COUNTS_PER_REV));
        lm.setTargetPosition((int) (rev * COUNTS_PER_REV));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(power);
        lm.setPower(power);
        while(rm.isBusy() && lm.isBusy()) {}
        stopMotor();
    }
    public void stopMotor() {
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


}

