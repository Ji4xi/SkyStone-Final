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
public class CalvinAutoEncoder extends LinearOpMode {

    protected BNO055IMU imu; //For detecting rotation

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;

    static final double COUNTS_PER_REVOLUTION = 1120;

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        for (int i = 0; i < 4; i++) {
            encoderStraight(DRIVE_SPEED, 5);
            encoderTurn(TURN_SPEED, 1.25, turnDirection.CLOCKWISE);
        }

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

    public void encoderStraight(double speed, double revolutions) {

        leftDrive.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));
        rightDrive.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {
            telemetry.addData("leftDrive", leftDrive.getPower());
            telemetry.addData("rightDrive", rightDrive.getPower());
            telemetry.update();
        }

        stopMotorsAndResetEncoders();

        sleep(1000);

    }

    public void encoderTurn(double speed, double revolutions, turnDirection direction) {
        if (direction == turnDirection.CLOCKWISE)
        {
            leftDrive.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));
            rightDrive.setTargetPosition(-(int) (revolutions * COUNTS_PER_REVOLUTION));
        }
        else if (direction == turnDirection.COUNTERCLOCKWISE)
        {
            leftDrive.setTargetPosition(-(int) (revolutions * COUNTS_PER_REVOLUTION));
            rightDrive.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        stopMotorsAndResetEncoders();

        sleep(1000);
    }

    public void stopMotorsAndResetEncoders() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum turnDirection {
        CLOCKWISE, COUNTERCLOCKWISE;
    }
}
