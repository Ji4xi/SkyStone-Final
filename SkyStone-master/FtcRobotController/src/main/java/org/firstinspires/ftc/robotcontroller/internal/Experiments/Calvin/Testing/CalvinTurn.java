package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@Autonomous
public class CalvinTurn extends LinearOpMode {

    protected BNO055IMU imu; //For detecting rotation

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;

    static final double COUNTS_PER_REVOLUTION = 1120;

    static final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        turnAbsolute(0.6, 90, 5);

    }

    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        stopMotorsAndResetEncoders();
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void turnAbsolute(double speed, double degrees, double error) { //positive degrees is counterclockwise, negative is clockwise
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double target = getAbsoluteHeading() + degrees;
        while (Math.abs(target - getAbsoluteHeading()) > error) {
            if (target - getAbsoluteHeading() > 0) {
                leftDrive.setPower(-((target-getAbsoluteHeading()) / target + 0.15) * speed);
                rightDrive.setPower(((target-getAbsoluteHeading()) / target + 0.15) * speed);
            }
            else {
                leftDrive.setPower(((target-getAbsoluteHeading()) / target + 0.15) * speed);
                rightDrive.setPower(-((target-getAbsoluteHeading()) / target + 0.15) * speed);
            }
        }

        stopMotorsAndResetEncoders();

        sleep(1000);
    }

    public void stopMotorsAndResetEncoders() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
