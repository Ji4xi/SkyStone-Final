package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PD;
import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class NewStoneFinder extends opencvSkystoneDetector {
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
    protected double gyroPwr = 0;

    protected enum SkystonePositions {LEFT, MID, RIGHT}
    //Field Variables
    protected final double TILE_INCH = 22.75;
    protected final double CONNECTION_INCH = 1.125;
    protected final double SKYSTONE_INCH = 8;
    protected final double ROBOT_LENGTH_INCH = 17.85;
    protected final double ROBOT_WIDTH_INCH = 17.7;

    PD pd = new PD(1, 1250);

    double skystoneAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
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
        foundation(Mode.CLOSE, 0);

        GlobalCoordinate globalCoordinate = new GlobalCoordinate(fl, fr, bl, br, imu);
        Thread globalCoordinateThread = new Thread(globalCoordinate);
        globalCoordinateThread.start();

        //find skystone pos
        super.runOpMode(); //init camera

        double mid = 0;

        while (!isStarted()) {
            if (valLeft == 0) {
                skystoneAngle = mid + 14;
            }
            else if (valMid == 0) {
                skystoneAngle = mid;
            }
            else {
                skystoneAngle = mid - 14;
            }

            telemetry.addData("angle", skystoneAngle);
            telemetry.update();
        }
        phoneCam.closeCameraDevice();
    }




    public void telemetry() {
        telemetry.addData("target", Math.abs(fr.getTargetPosition()) + Math.abs(fl.getTargetPosition()) + Math.abs(br.getTargetPosition()) + Math.abs(bl.getTargetPosition()));
        telemetry.addData("current", Math.abs(fr.getCurrentPosition()) + Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition()));
        telemetry.addData("fl_pwr", fl.getPower());
        telemetry.addData("fr_pwr", fr.getPower());
        telemetry.addData("bl_pwr", bl.getPower());
        telemetry.addData("br_pwr", br.getPower());
        telemetry.addData("fl_target_pos", fl.getTargetPosition());
        telemetry.addData("fr_target_pos", fr.getTargetPosition());
        telemetry.addData("bl__target_pos", bl.getTargetPosition());
        telemetry.addData("br_target_pos", br.getTargetPosition());


        telemetry.addData("fl_target_left", Math.abs(fl.getTargetPosition()) - Math.abs(fl.getCurrentPosition()));
        telemetry.addData("fr_target_left", Math.abs(fr.getTargetPosition()) - Math.abs(fr.getCurrentPosition()));
        telemetry.addData("br_target_left", Math.abs(br.getTargetPosition()) - Math.abs(br.getCurrentPosition()));
        telemetry.addData("bl_target_left", Math.abs(bl.getTargetPosition()) - Math.abs(bl.getCurrentPosition()));
        telemetry.addData("heading", getNormalizedHeading());
        telemetry.addData("section", section);
        telemetry.update();
    }

    public void moveInchesShort (double inches, double angle, double gyroAngle) {
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * COUNTS_PER_REVOLUTION);
        double snipPwr, currentPos;
        String section;
        double maxPwr = 0.4;

        fr.setTargetPosition((int) (transformation(angle, "fr") * target));
        fl.setTargetPosition((int) (transformation(angle, "fl") * target));
        br.setTargetPosition((int) (transformation(angle, "br") * target));
        bl.setTargetPosition((int) (transformation(angle, "bl") * target));

        target = Math.abs(fr.getTargetPosition()) + Math.abs(fl.getTargetPosition()) + Math.abs(br.getTargetPosition()) + Math.abs(bl.getTargetPosition()); //*2 for 2 motors

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((Math.abs(fr.getCurrentPosition()) + Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) < target - 15) {
            snipPwr = 0.4;
            fr.setPower(transformation(angle, "fr") * snipPwr + gyro(gyroAngle, "fr"));
            fl.setPower(transformation(angle, "fl") * snipPwr + gyro(gyroAngle, "fl"));
            br.setPower(transformation(angle, "br") * snipPwr + gyro(gyroAngle, "br"));
            bl.setPower(transformation(angle, "bl") * snipPwr + gyro(gyroAngle, "bl"));
            telemetry();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void moveInches (double inches, double angle, double gyroAngle) throws InterruptedException{
        double circ = Math.PI * (3.93701);
        int target = (int) (inches / circ * COUNTS_PER_REVOLUTION);
        double snipPwr = 0, currentPos;
        double maxPwr = 1;

        waitOneFullHardwareCycle();
        fr.setTargetPosition((int) (transformation(angle, "fr") * target));
        fl.setTargetPosition((int) (transformation(angle, "fl") * target));
        br.setTargetPosition((int) (transformation(angle, "br") * target));
        bl.setTargetPosition((int) (transformation(angle, "bl") * target));

        target = Math.abs(fr.getTargetPosition()) + Math.abs(fl.getTargetPosition()) + Math.abs(br.getTargetPosition()) + Math.abs(bl.getTargetPosition()); //*2 for 2 motors
        double front = (TILE_INCH / 2) * 4 / circ * COUNTS_PER_REVOLUTION;
        double end = target - (TILE_INCH) * 4 / circ * COUNTS_PER_REVOLUTION;
        double mid = (target - end) * 4;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while ((Math.abs(fr.getCurrentPosition()) + Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) < target - 15) {
            updateVelocity();
            currentPos = Math.abs(fr.getCurrentPosition()) + Math.abs(fl.getCurrentPosition()) + Math.abs(br.getCurrentPosition()) + Math.abs(bl.getCurrentPosition());
            if (currentPos < front) section = "front";
            else if (currentPos > end) section = "end";
            else section = "mid";

            switch (section) {
                case "front": snipPwr = currentPos / front * (maxPwr - 0.25) + 0.25; break;
                case "end": snipPwr = 0; gyroPwr = 0.1; break; //snipPwr = (1 - (currentPos -  end) / (target - end)) * (maxPwr - 0.1) + 0.1;
                case "mid": snipPwr = trueVelocity < 2490 ? snipPwr + 0.01 : snipPwr - 0.01; break; //0.8 for transition
                default: snipPwr = 0; //safety
            }

            fr.setPower(transformation(angle, "fr") * snipPwr + gyro(gyroAngle, "fr")* (snipPwr + gyroPwr));
            fl.setPower(transformation(angle, "fl") * snipPwr + gyro(gyroAngle, "fl")* (snipPwr + gyroPwr));
            br.setPower(transformation(angle, "br") * snipPwr + gyro(gyroAngle, "fr")* (snipPwr + gyroPwr));
            bl.setPower(transformation(angle, "bl") * snipPwr + gyro(gyroAngle, "bl")* (snipPwr + gyroPwr));
            telemetry();

            waitOneFullHardwareCycle();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        gyroPwr = 0;
    }
    public void updateVelocity() {
        double lastCount = fr.getCurrentPosition();
        if (fr.getTargetPosition() > fl.getTargetPosition()) {
            lastCount = fr.getCurrentPosition();
            time = System.nanoTime();
            double velocity = (fr.getCurrentPosition() - lastCount) / ((System.nanoTime() - time) * Math.pow(10, -9));
            double sum = 0;
            for (int i = 0; i < list.length - 1; i++) {
                list[i] = list[i + 1];
                sum += list[i];
            }
            list[list.length - 1] = velocity;
            trueVelocity = (sum + velocity) / list.length;
        } else {
            lastCount = fl.getCurrentPosition();
            time = System.nanoTime();
            double velocity = (fl.getCurrentPosition() - lastCount) / ((System.nanoTime() - time) * Math.pow(10, -9));
            double sum = 0;
            for (int i = 0; i < list.length - 1; i++) {
                list[i] = list[i + 1];
                sum += list[i];
            }
            list[list.length - 1] = velocity;
            trueVelocity = (sum + velocity) / list.length;
        }

    }
    public void reinitialize() {
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void runToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runToPosition(int target, double angle) {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runToPosition(int frtarget, int fltarget) {
        fl.setTargetPosition(fltarget);
        fr.setTargetPosition(frtarget);
        bl.setTargetPosition(frtarget);
        br.setTargetPosition(fltarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //angle in radians
    public double transformation (double angles, String motor) {
        //fr fl br bl
        angles = Math.toRadians(angles);
        switch (motor) {
            case "fr" : return Math.sqrt(2) * Math.sin(angles - Math.PI / 4) > 1 ? 1: Math.sqrt(2) * Math.sin(angles - Math.PI / 4);
            case "fl" : return Math.sqrt(2) * Math.cos(angles - Math.PI / 4) > 1 ? 1: Math.sqrt(2) * Math.cos(angles - Math.PI / 4);
            case "br" : return Math.sqrt(2) * Math.cos(angles - Math.PI / 4) > 1 ? 1: Math.sqrt(2) * Math.cos(angles - Math.PI / 4);
            case "bl" : return Math.sqrt(2) * Math.sin(angles - Math.PI / 4) > 1 ? 1: Math.sqrt(2) * Math.sin(angles - Math.PI / 4);
            default: return 0;
        }
    }
    public double gyro (double angles, String motor) {
        //fr fl br bl
        double a = 70; //adjusting rate
        double dif = angles - getNormalizedHeading();
        if (dif > 0) { //need counterclockwise
            switch (motor) {
                case "fr" : return dif / a;
                case "fl" : return - dif / a;
                case "br" : return dif / a;
                case "bl" : return - dif / a;
                default: return 0;
            }
        } else if (dif < 0) {
            switch (motor) {
                case "fr" : return dif / a;
                case "fl" : return - dif / a;
                case "br" : return dif / a;
                case "bl" : return - dif / a;
                default: return 0;
            }
        } else {
            return 0;
        }

    }


    public void moveClaw(double position1, double position2) {
        topClaw.setPosition(position1);
        bottomClaw.setPosition(position2);
    }

    public void flipMechanic(DcMotor... motors) {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i].getDirection() == DcMotorSimple.Direction.FORWARD) motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            else motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getNormalizedHeading() {
        return (getAbsoluteHeading() + 450) % 360;
    }
    public void foundation(Mode position, int sleep) throws InterruptedException {
        if (position == position.CLOSE) {
            ls.setPosition(0);
            rs.setPosition(0);
            sleep(sleep);
        }
        else if (position == position.OPEN) {
            ls.setPosition(0.5);
            rs.setPosition(0.5);
            sleep(sleep);
        }
    }
}
