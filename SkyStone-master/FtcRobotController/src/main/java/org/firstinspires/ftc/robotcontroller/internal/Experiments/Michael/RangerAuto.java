package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Disabled
@Autonomous(name = "RangerAuto")
public class RangerAuto extends SkystoneIdentificationSample {
    protected BNO055IMU imu; //For detecting rotation
    DcMotor rm;
    DcMotor lm;
    final double COUNTS_PER_REVOLUTION = 1120; //40:1
    double error = 5;
    PIDMichael pid = new PIDMichael(0.001, Math.pow(10, -12), 10000);
    enum Direction {
        CLOCKWISE, COUNTERCLOCKWISE
    }
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        moveForward(0.8, 4);
        sleep(1000);
        turnAbsolute(-90, Direction.CLOCKWISE, 0.5);
        turnAbsolute(90, Direction.COUNTERCLOCKWISE, 0.5);

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
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        lm.setDirection(DcMotorSimple.Direction.FORWARD);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        while(opModeIsActive() && rm.isBusy() && lm.isBusy()) {
            power = pid.actuator(rm.getCurrentPosition(), System.nanoTime()) * power;
            rm.setPower(power);
            lm.setPower(power);
            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.addData("target_left", target - rm.getCurrentPosition());
            telemetry.addData("p", pid.p * pid.error);
            telemetry.addData("i", pid.i * pid.integral);
            telemetry.addData("d", pid.d * pid.differential);
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

    public void turnAbsolute(double angle, Direction direction, double power) {
        double angleLeft = Math.abs(getAbsoluteHeading() - angle);
        double firstDiff = Math.abs(getAbsoluteHeading() - angle);
        double integral = 0.18;
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction == Direction.CLOCKWISE) {
            while (angleLeft > error) {
                angleLeft = Math.abs(getAbsoluteHeading() - angle);
                if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                else power = integral;
                rm.setPower(-power);
                lm.setPower(power);
                telemetry.addData("rm_pwr", rm.getPower());
                telemetry.addData("lm_pwr", lm.getPower());
                telemetry.addData("imu", getAbsoluteHeading());
                telemetry.addData("angleLeft", angleLeft);
                telemetry.addData("firstDiff", firstDiff);
                telemetry.addData("clockwise", "true");
                telemetry.update();
            }
        } else {
            while (angleLeft > error) {
                angleLeft = Math.abs(getAbsoluteHeading() - angle);
                if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                else power = integral;
                rm.setPower(power);
                lm.setPower(-power);
                telemetry.addData("rm_pwr", rm.getPower());
                telemetry.addData("lm_pwr", lm.getPower());
                telemetry.addData("imu", getAbsoluteHeading());
                telemetry.addData("angleLeft", angleLeft);
                telemetry.addData("firstDiff", firstDiff);
                telemetry.addData("clockwise", "true");
                telemetry.update();
            }
        }
    }
//    public void turnAbsolute(double power, int angles) {
//        // |angle|
//        //desired power 0.1
//        double extraPush = 0.1;
//        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("angle on imu", getAbsoluteHeading());
//        telemetry.addData("angle target", angles);
//        telemetry.addData("angle left", Math.abs(getAbsoluteHeading() - angles));
//        telemetry.update();
//        boolean key = true;
//        if (Math.abs(angles) > 180) {
//            if (angles < 0) {
//                angles += 360;
//            } else {
//                angles -= 360;
//            }
//            key = false;
//        }
//        while (Math.abs(getAbsoluteHeading() - angles) > error) {
//            power = Math.abs(getAbsoluteHeading() - angles) / Math.abs(angles) + extraPush;
//            if (key) {
//                if (angles > 0) {
//                    rm.setPower(power);
//                    lm.setPower(-power);
//                } else {
//                    rm.setPower(-power);
//                    lm.setPower(power);
//                }
//            } else {
//                if (angles > 0) {
//                    rm.setPower(-power);
//                    lm.setPower(power);
//                } else {
//                    rm.setPower(power);
//                    lm.setPower(-power);
//                }
//            }
//        }
//        stopMotor();
//    }

    public double getAbsoluteHeading() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return angle;
    }

}
