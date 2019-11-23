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
public class CalvinAutoTest extends LinearOpMode {

    protected BNO055IMU imu; //For detecting rotation

    DcMotor leftDrive, rightDrive;

    static final double COUNTS_PER_MOTOR_REV = 1120;

    static final double DRIVE_POWER = 0.8;
    static final double TURN_POWER = 0.2;

    static double globalDegrees = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        for (int i = 0; i < 4; i++) {
            driveRev(5, DRIVE_POWER);
            turnAbsolute(90, TURN_POWER, 5);
        }
    }

    public void initialize() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        stopMotorsResetEncoders();
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void driveRev(double revolutions, double power) {
        leftDrive.setTargetPosition((int) (revolutions * COUNTS_PER_MOTOR_REV));
        rightDrive.setTargetPosition((int) (revolutions * COUNTS_PER_MOTOR_REV));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {
            telemetry.addData("leftDrive power", leftDrive.getPower());
            telemetry.addData("rightDrive power", rightDrive.getPower());
            telemetry.update();
        }

        stopMotorsResetEncoders();
    }

    public void turnAbsolute(double degrees, double power, double error) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(degrees - getAbsoluteHeading()) > error) {
            if (degrees - getAbsoluteHeading() > 0) {
                leftDrive.setPower(-power);
                rightDrive.setPower(power);
            }
            else if (degrees - getAbsoluteHeading() < 0) {
                leftDrive.setPower(power);
                rightDrive.setPower(-power);
            }
        }

        stopMotorsResetEncoders();
        globalDegrees = degrees;
    }

    public void stopMotorsResetEncoders() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
