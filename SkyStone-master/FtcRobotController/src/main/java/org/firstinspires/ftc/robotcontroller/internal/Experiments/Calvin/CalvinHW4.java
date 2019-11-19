package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

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
@Autonomous
public class CalvinHW4 extends LinearOpMode {

    protected BNO055IMU imu; //For detecting rotation

    public void initializeIMU() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    DcMotor ld;
    DcMotor rd;

    static final double COUNTS_PER_REVOLUTION = 1120;

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        ld = hardwareMap.dcMotor.get("ld");
        rd = hardwareMap.dcMotor.get("rd");
        ld.setDirection(DcMotorSimple.Direction.REVERSE);
        rd.setDirection(DcMotorSimple.Direction.FORWARD);

        ld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize imu
        initializeIMU();

        waitForStart();

        //first forward move
        drive(DRIVE_SPEED, 5);

        //turn 270 degrees counterclockwise
        while (getAbsoluteHeading() <= 270) {

            ld.setPower((-TURN_SPEED * (270 - getAbsoluteHeading())) + 0.08);
            rd.setPower((TURN_SPEED * (270 - getAbsoluteHeading())) + 0.08);

            telemetry.addData("angle", getAbsoluteHeading());

            telemetry.update();
        }

        ld.setPower(0);
        rd.setPower(0);

        ld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //second forward move
        drive(DRIVE_SPEED, 3);

    }

    public void drive(double speed, double revolutions) {

        ld.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));
        rd.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));

        ld.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ld.setPower(speed);
        rd.setPower(speed);

        while (ld.isBusy() && rd.isBusy()) {
            telemetry.addData("left", ld.getCurrentPosition());
            telemetry.addData("right", rd.getCurrentPosition());

            telemetry.update();
        }

        ld.setPower(0);
        rd.setPower(0);

        ld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
