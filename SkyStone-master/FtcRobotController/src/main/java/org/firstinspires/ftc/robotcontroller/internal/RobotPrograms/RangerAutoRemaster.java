package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous
public class RangerAutoRemaster extends LinearOpMode {

    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    final double COUNTS_PER_REV = 1120;
    double wheelDiameter = 3;
    double wheelCir = wheelDiameter * Math.PI;
    final int error = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        driveTest(15, 0.4);
        sleep(1000);
    }
    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
        //drive
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm = hardwareMap.dcMotor.get("lm");
        lm.setDirection(DcMotorSimple.Direction.FORWARD);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void useEncoder() {
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveTest (double inches, double power) {
        int target = (int) ((inches / wheelCir) * COUNTS_PER_REV);
        useEncoder();
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (rm.isBusy() && lm.isBusy()) {
            rm.setPower(power);
            lm.setPower(power);
            int restrain = (int) (0 - getAbsoluteHeading());
            if (restrain > error) {
                rm.setPower(0.3);
                lm.setPower(-0.3);
            }
            if (restrain < error) {
                rm.setPower(-0.3);
                lm.setPower(0.3);
            }

            else {
                rm.setPower(power);
                lm.setPower(power);
            }
        }
        stopMotor();
    }

    public void stopMotor() {
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnTest( double power) {

        int turnTarget = (int) (1 * COUNTS_PER_REV);
        rm.setTargetPosition(turnTarget);
        lm.setTargetPosition(turnTarget);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setPower(power);
        lm.setPower(-power);

    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}

