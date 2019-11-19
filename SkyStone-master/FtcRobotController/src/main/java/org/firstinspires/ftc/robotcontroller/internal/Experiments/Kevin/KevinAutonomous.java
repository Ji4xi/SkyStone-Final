package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PID;
import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.ArcadeDrive;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.DriveTrain;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.EncoderMode;
import org.firstinspires.ftc.robotcontroller.internal.TestProps.Arcade;
import org.firstinspires.ftc.robotcontroller.internal.TestProps.OneServoOneMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name = "KevinAutonomous")
public class KevinAutonomous extends LinearOpMode {
    protected BNO055IMU imu;
    double error = 5;
    DcMotor rightMotor;
    DcMotor leftMotor;
    @Override
    public void runOpMode () throws InterruptedException {
        final double COUNTS_PER_REVELATION = 1120;
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        initializeIMU();
        waitForStart();
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setTargetPosition((int) (2 * COUNTS_PER_REVELATION));
        leftMotor.setTargetPosition((int) (2 * COUNTS_PER_REVELATION));

        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor.setPower(0.5);
        leftMotor.setPower(0.5);
        while (rightMotor.isBusy() && leftMotor.isBusy()) {
            telemetry.addData("left", leftMotor.getPower());
            telemetry.addData("right", rightMotor.getPower());
            telemetry.update();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);

        rightMotor.setTargetPosition((int) (3 * COUNTS_PER_REVELATION));
        leftMotor.setTargetPosition((int) (-3 * COUNTS_PER_REVELATION));

        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor.setPower(0.5);
        leftMotor.setPower(0.5);
        while (rightMotor.isBusy() && leftMotor.isBusy()) {
            telemetry.addData("left2", leftMotor.getPower());
            telemetry.addData("right2", rightMotor.getPower());
            telemetry.update();
        }


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition((int) (3 * COUNTS_PER_REVELATION));
        rightMotor.setTargetPosition((int) (3 * COUNTS_PER_REVELATION));

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor.setPower(0.5);
        leftMotor.setPower(0.5);
        while(rightMotor.isBusy() && leftMotor.isBusy()){
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
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
    public void turn(double angles, boolean clockwise, double power) {
        telemetry.addData("angle on imu", getAbsoluteHeading());
        telemetry.addData("angle target", angles);
        telemetry.addData("angle left", getAbsoluteHeading() - angles);
        telemetry.update();
        angles = Math.abs(angles);
        if (Math.abs(getAbsoluteHeading() - angles) > error) {
            if (clockwise) {
                rightMotor.setPower(power);
                leftMotor.setPower(-power);
            } else {
                rightMotor.setPower(-power);
                leftMotor.setPower(power);
            }
            turn(angles, clockwise, power);
        } else {
            stop();
        }
    }
}

