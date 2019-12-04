package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

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
@Autonomous

public class AutoTest extends LinearOpMode {
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    final double COUNTS_PER_REVOLUTION = 1120;
    double error = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //phase 1
        moveForward(0.5, 10);
        sleep(1000);
        //phase 2
        turnAbsolute(0.2, 90 );
        sleep(1000);
        //phase 3
        moveForward(0.5, 5);
        sleep(1000);
    }

    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        rm = hardwareMap.dcMotor.get("rm");
        lm = hardwareMap.dcMotor.get("lm");
        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void moveForward (double power, double revolutions) {
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(power);
        lm.setPower(power);
        while (rm.isBusy()) {
        }
        stopMotor();
        moveForward(power, revolutions);
    }
    public void stopMotor() {
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnAbsolute(double power, int degrees) {
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(getAbsoluteHeading() - degrees) > error) {
            if (degrees > 0) {
                rm.setPower(power);
                lm.setPower(-power);
            }

            else if (degrees > 0) {
                rm.setPower(-power);
                lm.setPower(power);
            }
            else {
               stopMotor();
            }
            turnAbsolute(power, degrees);
        }
        else {
            stopMotor();
        }

    }
}
