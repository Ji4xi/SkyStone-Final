package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.PD;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.GlobalCoordinate;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.StoneFinder;

@Autonomous
public class OdometryTesting extends LinearOpMode {
    double turnPwrMax = 0.3;

    protected BNO055IMU imu;//For detecting angles of rotation
    protected String section;
    protected enum Mode {OPEN, CLOSE}
    protected enum Side {LEFT, RIGHT}
    protected enum GAGE {FORWARD, REVERSE}

    //driveTrain
    protected DcMotor fl;
    protected DcMotor fr;
    protected DcMotor bl;
    protected DcMotor br;

    protected double[] list = new double[100];
    protected double trueVelocity;
    protected double time = System.nanoTime();


    protected DcMotor rightIntake;
    protected DcMotor leftIntake;

    protected Servo hook;

    protected Servo rs;
    protected Servo ls;

    protected CRServo tape;

    protected DcMotor rightLift;
    protected DcMotor leftLift;

    protected Servo topClaw;
    protected Servo bottomClaw;

    protected double drivePwrMax = 0.3;
    protected final double intakePwr = 0.5;
    protected final double maxLiftPwr = 0.6;
    protected double currentLiftPwr = 0;
    protected static final double COUNTS_PER_REVOLUTION = 537.6; //20:1
    protected static final double DRIVE_GEAR_REDUCTION = 1; //This is < 1.0 if geared up
    protected static final double WHEEL_DIAMETER_INCHES = 3.93701;
    protected static final double WHEEL_PERIMETER_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    protected enum Direction {CLOCKWISE, COUNTERCLOCKWISE}


    protected enum SkystonePositions {LEFT, MID, RIGHT}
    //Field Variables
    protected final double TILE_INCH = 22.75;
    protected final double CONNECTION_INCH = 1.125;
    protected final double SKYSTONE_INCH = 8;
    protected final double ROBOT_LENGTH_INCH = 17.85;
    protected final double ROBOT_WIDTH_INCH = 17.7;

    PD pd = new PD(1, 1250);
    GlobalCoordinate globalCoordinate;

    double skystoneAngle;
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        globalCoordinate = new GlobalCoordinate(fl, fr, imu);
        Thread globalCoordinateThread = new Thread(globalCoordinate);
        globalCoordinateThread.start();
        while (opModeIsActive()) {
            telemetry.addData("globalX", globalCoordinate.getGlobalX() / COUNTS_PER_INCH);
            telemetry.addData("globalY", globalCoordinate.getGlobalY() / COUNTS_PER_INCH);
            telemetry.addData("changeHorizonal", globalCoordinate.getChangeHorizontal());
            telemetry.addData("changeVertical", globalCoordinate.getChangeVertical());

            telemetry.addData("fr_encoder_count", fr.getCurrentPosition());
            telemetry.addData("fl_encoder_count", fl.getCurrentPosition());
            telemetry.addData("br_encoder_count", br.getCurrentPosition());
            telemetry.addData("bl_encoder_count", bl.getCurrentPosition());

            telemetry.update();

        }
        globalCoordinateThread.stop();
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



        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }

    public void updateDriveTrain() {

                fl.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-
                fr.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
                bl.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
                br.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-


    }
}
