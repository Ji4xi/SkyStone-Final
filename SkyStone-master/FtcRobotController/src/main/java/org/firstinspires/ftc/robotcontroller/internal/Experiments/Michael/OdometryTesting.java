package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.GlobalCoordinate;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.StoneFinder;

@Autonomous
public class OdometryTesting extends NewStoneFinder {
    protected double turnPwrMax = 0.60;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
    }

    @Override
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

        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);

        topClaw = hardwareMap.servo.get("top");
        bottomClaw = hardwareMap.servo.get("bot");
        topClaw.setDirection(Servo.Direction.REVERSE);
        bottomClaw.setDirection(Servo.Direction.REVERSE);

        tape = hardwareMap.crservo.get("tape");
        tape.setDirection(CRServo.Direction.REVERSE);

        topClaw.setPosition(0.4);
        bottomClaw.setPosition(0.3);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foundation(Mode.CLOSE, 0);

        GlobalCoordinate globalCoordinate = new GlobalCoordinate(fl, fr, imu);
        Thread globalCoordinateThread = new Thread(globalCoordinate);
        globalCoordinateThread.start();

        while (opModeIsActive()) {
            telemetry.addData("globalX", globalCoordinate.getGlobalX());
            telemetry.addData("globalY", globalCoordinate.getGlobalY());
            telemetry.addData("changeHorizonal", globalCoordinate.getChangeHorizontal());
            telemetry.addData("changeVertical", globalCoordinate.getChangeVertical());
        }


    }

    public void updateDriveTrain() {

                fl.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-
                fr.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
                bl.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
                br.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-


    }
}
