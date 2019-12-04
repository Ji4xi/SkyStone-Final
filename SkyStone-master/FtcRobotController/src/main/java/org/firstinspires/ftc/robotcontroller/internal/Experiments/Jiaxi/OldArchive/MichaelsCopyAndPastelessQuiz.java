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
import org.firstinspires.ftc.robotserver.internal.webserver.controlhubupdater.ChUpdaterCommManager;

@Disabled
@Autonomous
public class MichaelsCopyAndPastelessQuiz extends LinearOpMode {
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    double error = 5;
    final double COUNTS_PER_REVOLUTION = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //phase 1
        phase();
        //phase 2
        phase();
        //phase 3
        phase();
        //phase 4
        phase();
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
    public void moveForward(double power, double rev) {
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition((int) (rev * COUNTS_PER_REVOLUTION));
        lm.setTargetPosition((int) (rev * COUNTS_PER_REVOLUTION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(power);
        rm.setPower(power);
        while (rm.isBusy()) {
        }
        stopMotor();
    }
    public void stopMotor() {
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnAbsolute(double power, double angle) {
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(getAbsoluteHeading() - angle ) > error) {
            if (angle > 0) {
                rm.setPower(power);
                lm.setPower(-power);
            }
            else if (angle < 0) {
                rm.setPower(-power);
                lm.setPower(power);
            }
            turnAbsolute(power, angle);
        }
        else {
            stopMotor();
        }
    }
    public void phase() {
        moveForward(0.5,5);
        sleep(1000);
        turnAbsolute(0.2, -90);
        sleep(1000);
    }
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
