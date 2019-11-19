package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Disabled
@Autonomous(name = "AutoAssessement")
public class AutoAssessement extends LinearOpMode {
    protected BNO055IMU imu; //For detecting rotation
    DcMotor rm;
    DcMotor lm;
    double error = 5;
    double maxPwr = 0.6;
    final double COUNTS_PER_REVOLUTION = 1120; //40:1
    PIDMichael pid = new PIDMichael(1, 0.003, 0.003);
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //state 1
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

    }
    public void turnAbsolute(double power, int angles) {
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("angle on imu", getAbsoluteHeading());
        telemetry.addData("angle target", angles);
        telemetry.addData("angle left", Math.abs(getAbsoluteHeading() - angles));
        telemetry.update();
        if (Math.abs(getAbsoluteHeading() - angles) > error) {
            if (angles > 0) {
                rm.setPower(power);
                lm.setPower(-power);
            } else {
                rm.setPower(-power);
                lm.setPower(power);
            }
            turnAbsolute(power, angles);
        } else {
            stopMotor();
        }
    }
    public void moveForward(double power, double rev) {
        int target = (int) (rev * COUNTS_PER_REVOLUTION);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pid.initPID(target, System.nanoTime());
        while(rm.isBusy() && lm.isBusy()) {
            power = pid.actuator(rm.getCurrentPosition(), System.nanoTime()) * maxPwr;
            rm.setPower(power);
            lm.setPower(power);
            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("target_left", target - rm.getCurrentPosition());
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

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
