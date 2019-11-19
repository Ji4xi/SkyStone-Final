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
//@Disabled
@Autonomous
public class KevinAutonomousTest extends LinearOpMode {
    protected BNO055IMU imu;
    DcMotor rm;
    DcMotor lm;
    double error = 5;
    final double COUNTS_PER_REVELATION = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        //phase 1
        moveForward(0.69, 6.9);
        sleep(4200);
        //phase 2
        turnAbsolute(0.69, -69);
        sleep(4200);
        //phase 3
        moveForward(0.69420, 4.2);
        sleep(690);
        //phase 4
        turnAbsolute(0.69, 360);
        sleep(1000);
    }

    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
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
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnAbsolute(double power, double angles) {
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void moveForward (double power, double rev){
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition((int) (rev * COUNTS_PER_REVELATION));
        lm.setTargetPosition((int) (rev * COUNTS_PER_REVELATION));
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setPower(power);
        lm.setPower(power);
        while (lm.isBusy() && rm.isBusy()){
            stopMotor();
        }
    }
    public void stopMotor (){
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
