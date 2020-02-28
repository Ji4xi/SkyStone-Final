package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class MyOdometryOpmode extends LinearOpMode {
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
        globalCoordinate = new GlobalCoordinate(fl, fr, bl, br, imu);
        Thread globalCoordinateThread = new Thread(globalCoordinate);
        globalCoordinateThread.start();
        telemetry.addData("init finished", ">>");
        telemetry.update();
        waitForStart();
        goToPosition(0,40,0.3,90);
        sleep(3000);
        goToPosition(10,40,0.3,90);
        //globalCoordinateThread.stop();
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double gyroAngle) {
        double allowableDistanceError = 1 * COUNTS_PER_INCH;
        double angle, gyroPwr = 0;
        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
        double distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
        double snipPwr = robotPower;
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && distance > allowableDistanceError) {

            distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
            distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
            angle = Math.toDegrees(Math.atan2(distanceToYTarget, distanceToXTarget));
            fr.setPower(transformation(angle, "fr") * snipPwr + gyro(gyroAngle, "fr") * (snipPwr + gyroPwr));
            fl.setPower(transformation(angle, "fl") * snipPwr + gyro(gyroAngle, "fl") * (snipPwr + gyroPwr));
            br.setPower(transformation(angle, "br") * snipPwr + gyro(gyroAngle, "br") * (snipPwr + gyroPwr));
            bl.setPower(transformation(angle, "bl") * snipPwr + gyro(gyroAngle, "bl") * (snipPwr + gyroPwr));
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            Object[] names = {"distance", "distance to X", "distance to Y"};
            Object[] numbers = {distance, distanceToXTarget, distanceToYTarget};
            Object[] names2 = {"theta"};
            Object[] numbers2 = {angle};
            telemetry(names,numbers,names2, numbers2);
//            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToYTarget, distanceToXTarget));
//            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
//            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
//            double pivot_correction = desiredRobotOrientation - globalCoordinate.getNormalizedHeading();

        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }


    public void telemetry(Object[]... maps) {
        for (int number = 0; number < maps[0].length; number++) {
            telemetry.addData(maps[0][number].toString(), maps[1][number].toString());
        }
        for (int number = 0; number < maps[2].length; number++) {
            telemetry.addData(maps[2][number].toString(), maps[3][number].toString());
        }

        telemetry.addData("globalX", globalCoordinate.getGlobalX() / COUNTS_PER_INCH);
        telemetry.addData("globalY", globalCoordinate.getGlobalY() / COUNTS_PER_INCH);
        telemetry.addData("changeHorizonal", globalCoordinate.getChangeHorizontal());
        telemetry.addData("changeVertical", globalCoordinate.getChangeVertical());

        telemetry.addData("fr_encoder_count", fr.getCurrentPosition());
        telemetry.addData("fl_encoder_count", fl.getCurrentPosition());
        telemetry.addData("br_encoder_count", br.getCurrentPosition());
        telemetry.addData("bl_encoder_count", bl.getCurrentPosition());

        telemetry.addData("fr_pwr", fr.getPower());
        telemetry.addData("fl_pwr", fl.getPower());
        telemetry.addData("br_pwr", br.getPower());
        telemetry.addData("bl_pwr", bl.getPower());
        telemetry.update();

    }

    public void moveInchesX(double targetXPosition, double allowableError, double angle, double gyroAngle) {
        double currentX = globalCoordinate.getGlobalX();
        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - currentX;
        double snipPwr = 0, gyroPwr = 0, maxPwr = 1;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (Math.abs(distanceToXTarget) > allowableError) {
            updateVelocity();
            if (distanceToXTarget > (targetXPosition - (0.5 * TILE_INCH) * COUNTS_PER_INCH))
                section = "front";
            else if (distanceToXTarget < (TILE_INCH) * COUNTS_PER_INCH) section = "end";
            else section = "mid";

            switch (section) {
                case "front":
                    snipPwr = distanceToXTarget / targetXPosition * (maxPwr - 0.25) + 0.25;
                    break;
                case "mid":
                    snipPwr = trueVelocity < 2490 ? snipPwr + 0.01 : snipPwr - 0.01;
                    break; //0.8 for transition
                case "end":
                    snipPwr = 0;
                    gyroPwr = 0.1;
                    break; //snipPwr = (1 - (currentPos -  end) / (target - end)) * (maxPwr - 0.1) + 0.1;
                default:
                    snipPwr = 0; //safety
            }

            fr.setPower(transformation(angle, "fr") * snipPwr + gyro(gyroAngle, "fr") * (snipPwr + gyroPwr));
            fl.setPower(transformation(angle, "fl") * snipPwr + gyro(gyroAngle, "fl") * (snipPwr + gyroPwr));
            br.setPower(transformation(angle, "br") * snipPwr + gyro(gyroAngle, "fr") * (snipPwr + gyroPwr));
            bl.setPower(transformation(angle, "bl") * snipPwr + gyro(gyroAngle, "bl") * (snipPwr + gyroPwr));
            telemetry();

        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void moveInchesY(double targetYPosition, double allowableError, double angle, double gyroAngle) {
        double currentY = globalCoordinate.getGlobalY();
        double distanceToXTarget = targetYPosition * COUNTS_PER_INCH - currentY;
        double snipPwr = 0, gyroPwr = 0, maxPwr = 1;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (Math.abs(distanceToXTarget) > allowableError) {
            updateVelocity();
            if (distanceToXTarget > (targetYPosition - (0.5 * TILE_INCH) * COUNTS_PER_INCH))
                section = "front";
            else if (distanceToXTarget < (TILE_INCH) * COUNTS_PER_INCH) section = "end";
            else section = "mid";

            switch (section) {
                case "front":
                    snipPwr = distanceToXTarget / targetYPosition * (maxPwr - 0.25) + 0.25;
                    break;
                case "mid":
                    snipPwr = trueVelocity < 2490 ? snipPwr + 0.01 : snipPwr - 0.01;
                    break; //0.8 for transition
                case "end":
                    snipPwr = 0;
                    gyroPwr = 0.1;
                    break; //snipPwr = (1 - (currentPos -  end) / (target - end)) * (maxPwr - 0.1) + 0.1;
                default:
                    snipPwr = 0; //safety
            }

            fr.setPower(transformation(angle, "fr") * snipPwr + gyro(gyroAngle, "fr") * (snipPwr + gyroPwr));
            fl.setPower(transformation(angle, "fl") * snipPwr + gyro(gyroAngle, "fl") * (snipPwr + gyroPwr));
            br.setPower(transformation(angle, "br") * snipPwr + gyro(gyroAngle, "fr") * (snipPwr + gyroPwr));
            bl.setPower(transformation(angle, "bl") * snipPwr + gyro(gyroAngle, "bl") * (snipPwr + gyroPwr));
            telemetry();

        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }


    public void moveInchesShortX(double targetXPosition, double allowableError, double angle, double gyroAngle) {
        double snipPwr;
        double distanceToXTarget = targetXPosition - globalCoordinate.getGlobalX();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(distanceToXTarget) > allowableError) {
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

    public void moveInchesShortY(double targetYPosition, double allowableError, double angle, double gyroAngle) {
        double snipPwr;
        double distanceToYTarget = targetYPosition - globalCoordinate.getGlobalY();
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (distanceToYTarget > allowableError) {
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

    public double gyro(double angles, String motor) {
        //fr fl br bl
        double a = 70; //adjusting rate
        double dif = angles - getNormalizedHeading();
        if (dif > 0) { //need counterclockwise
            switch (motor) {
                case "fr":
                    return dif / a;
                case "fl":
                    return -dif / a;
                case "br":
                    return dif / a;
                case "bl":
                    return -dif / a;
                default:
                    return 0;
            }
        } else if (dif < 0) {
            switch (motor) {
                case "fr":
                    return dif / a;
                case "fl":
                    return -dif / a;
                case "br":
                    return dif / a;
                case "bl":
                    return -dif / a;
                default:
                    return 0;
            }
        } else {
            return 0;
        }

    }

    //angle in radians
    public double transformation(double angles, String motor) {
        //fr fl br bl
        angles = Math.toRadians(angles);
        switch (motor) {
            case "fr":
                return Math.sin(angles - Math.PI / 4);
            case "fl":
                return Math.cos(angles - Math.PI / 4);
            case "br":
                return Math.cos(angles - Math.PI / 4);
            case "bl":
                return Math.sin(angles - Math.PI / 4);
            default:
                return 0;
        }
    }

    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public double getNormalizedHeading() {
        return (getAbsoluteHeading() + 450) % 360;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
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
}
