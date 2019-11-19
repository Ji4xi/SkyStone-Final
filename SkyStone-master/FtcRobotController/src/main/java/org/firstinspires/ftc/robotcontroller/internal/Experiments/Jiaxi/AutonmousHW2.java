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
@Autonomous(name = "JiaxiAuto")
public class AutonmousHW2 extends LinearOpMode {
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    @Override
    public void runOpMode () throws InterruptedException {
        final double COUNTS_PER_REVOLUTION = 1120;
        lm = hardwareMap.dcMotor.get("lm");
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        initializeIMU();
        //Phase 1
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        rm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        lm.setPower(0.5);
        rm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 2
        rm.setTargetPosition((int) (-1.25 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (1.25 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 3
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 4
        rm.setTargetPosition((int) (-1.25 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (1.25 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 5
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 6
        rm.setTargetPosition((int) (-1.25 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (1.25 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 7
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (5 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        //Phase 8
        rm.setTargetPosition((int) (-1.25 * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (1.25 * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(0.5);
        lm.setPower(0.5);
        while (rm.isBusy() && lm.isBusy()) {
        }
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);


    }
    public void initializeIMU(){
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }
    public double getAbsoluteHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
