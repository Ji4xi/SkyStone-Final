package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

//@Disabled
@Autonomous
public class newbot extends LinearOpMode {

    protected BNO055IMU imu; //For detecting angles of rotation

    //driveTrain
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    //foundation
    Servo rs;
    Servo ls;


    double drivePwrMax = 0.4;

    static final double COUNTS_PER_REVOLUTION = 537.6; //20:1
    static final double DRIVE_GEAR_REDUCTION = 1; //This is < 1.0 if geared up
    static final double WHEEL_DIAMETER_INCHES = 4.25;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        moveForwardInches(36, 5000);
        moveForwardInches(-36, 5000);
        moveSideways(3, 5000);
        moveSideways(-3, 5000);

    }

    public void initialize() throws InterruptedException {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //driveTrain
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveForwardInches(double inches, long sleep) throws InterruptedException {
        double power = drivePwrMax;
        int target = (int) (inches * COUNTS_PER_INCH);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setTargetPosition(target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(target);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() &&
                (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

            if (inches < 0) {
                fl.setPower(-power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(-power);
            } else {
                fl.setPower(power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(power);
            }

            telemetry.addData("fl_pwr", fl.getPower());
            telemetry.addData("fr_pwr", fr.getPower());
            telemetry.addData("bl_pwr", bl.getPower());
            telemetry.addData("br_pwr", br.getPower());
            telemetry.addData("target_left", target - (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition())/4);
            telemetry.update();
        }
        stopMotor();
        sleep(sleep, "moveForwardInches " + inches + " inches");
    }

    public void moveSideways(double revolutions, long sleep) throws InterruptedException {
        double power = drivePwrMax;
        int target = (int) (revolutions * COUNTS_PER_REVOLUTION);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setTargetPosition(target);
        fr.setTargetPosition(-target);
        bl.setTargetPosition(-target);
        br.setTargetPosition(target);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() &&
                (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

            if (revolutions < 0) {
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
            }
            else {
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(-power);
                br.setPower(power);
            }

            telemetry.addData("fl_pwr", fl.getPower());
            telemetry.addData("fr_pwr", fr.getPower());
            telemetry.addData("bl_pwr", bl.getPower());
            telemetry.addData("br_pwr", br.getPower());
            telemetry.addData("fl_target_left", target - fl.getCurrentPosition());
            telemetry.update();
        }
        stopMotor();
        sleep(sleep, "moveSideways " + revolutions + " revolutions");
    }

    public void stopMotor() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public final void sleep(long time, String input) {
        telemetry.addData("State Finished", input);
        telemetry.update();
        super.sleep(time);
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}
