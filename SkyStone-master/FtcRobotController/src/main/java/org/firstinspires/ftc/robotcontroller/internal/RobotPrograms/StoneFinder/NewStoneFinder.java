package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PD;
import org.firstinspires.ftc.robotcontroller.internal.Default.PID;
import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.CalvinPID;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class NewStoneFinder extends opencvSkystoneDetector {

    PID pid;
    protected boolean mode = true;
    protected double previousXTarget, previousYTarget;

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

    public enum Direction {CLOCKWISE, COUNTERCLOCKWISE}
    protected double gyroPwr = 0;
    protected GlobalCoordinate globalCoordinate;
    protected enum SkystonePositions {LEFT, MID, RIGHT}
    //Field Variables
    protected final double TILE_INCH = 22.75;
    protected final double CONNECTION_INCH = 1.125;
    protected final double SKYSTONE_INCH = 8;
    protected final double ROBOT_LENGTH_INCH = 17.85;
    protected final double ROBOT_WIDTH_INCH = 17.7;

    protected Thread globalCoordinateThread;
    public PD pd = new PD(0.001, 0, 0); //I: 0.001/ D: 0.0002/2.5 P?0.00067 0.00074 .00083//march 1st: 0.00064, 0.0000004, 0.000027
    CalvinPID PID = new CalvinPID(0.0000, 0.00000, 0.00000); //P: 0.012, I: 0.001, D: 0.0022

    double greatestDContribution = 0;
    double greatestIContribution = 0;

    public double skystoneAngle;
    public double maxPwr = 0.75;

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
        topClaw.setDirection(Servo.Direction.FORWARD);
        bottomClaw.setDirection(Servo.Direction.FORWARD);

        tape = hardwareMap.crservo.get("tape");
        tape.setDirection(CRServo.Direction.REVERSE);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        foundation(Mode.CLOSE, 0);

        bottomClaw.setPosition(0.21);
        topClaw.setPosition(0.47);
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        hook = hardwareMap.servo.get("hook");
        hook.setDirection(Servo.Direction.FORWARD);
        hook.setPosition(0.8);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        globalCoordinate = new GlobalCoordinate(fl, fr, bl, br, imu);
        globalCoordinateThread = new Thread(globalCoordinate);
        globalCoordinateThread.start();

        //find skystone pos
        super.runOpMode(); //init camera

        double mid = 0;

        while (!isStarted()) {
            if (valLeft == 0) {
                skystoneAngle = mid - 14;
            }
            else if (valMid == 0) {
                skystoneAngle = mid;
            }
            else {
                skystoneAngle = mid + 14;
            }

            telemetry.addData("angle", skystoneAngle);
            telemetry.update();
        }
        phoneCam.closeCameraDevice();
    }



    public void  goToPositionHard (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError) throws InterruptedException{
//        double circ = Math.PI * (3.93701);
//        targetXPosition = mode ? targetXPosition : -targetXPosition;
//
//        double angle, gyroPwr = 1;
//        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
//        double distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
//        double snipPwr = robotPower;
//        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
//        double target = distance;
//
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
////        pd.initPD(distance);
//
//
//        while (opModeIsActive() && distance > allowableDistanceError * COUNTS_PER_INCH) {
//            double normalHeading = getNormalizedHeading();
//            distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
//            distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
//            angle = Math.toDegrees(Math.atan2(distanceToYTarget, distanceToXTarget));
//            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
//
//            snipPwr = robotPower;
//            fr.setPower(transformation((angle) + 90 - normalHeading, "fr") * snipPwr + gyro(gyroAngle, "fr") * (gyroPwr) + pd.getDContrb());
//            fl.setPower(transformation((angle) + 90 - normalHeading, "fl") * snipPwr + gyro(gyroAngle, "fl") * (gyroPwr) + pd.getDContrb());
//            br.setPower(transformation((angle) + 90 - normalHeading, "br") * snipPwr + gyro(gyroAngle, "br") * (gyroPwr) + pd.getDContrb());
//            bl.setPower(transformation((angle) + 90 - normalHeading, "bl") * snipPwr + gyro(gyroAngle, "bl") * (gyroPwr) + pd.getDContrb());
//
////            Object[] names = {"gyro_con", "d", "distance", "distance to X", "distance to Y","angle","2angle-heading"};
//            Object[] names = {"p", 'i', 'd', "greatest Y"};
////            Object[] values = {pd.getPContrb(), pd.getIContrb(), pd.getDContrb()};
////            if (Math.abs(pd.getDContrb()) > Math.abs(greatestDContribution)) {greatestDContribution = pd.getDContrb();}
//            Object[] values = {pd.getPContrb(), pd.getIContrb(), pd.getDContrb(),greatestY /COUNTS_PER_INCH};
//            if (globalCoordinate.getGlobalY() > greatestY) {greatestY = globalCoordinate.getGlobalY();}
////            if (PID.getIntegral() > greatestIContribution) {greatestDContribution = PID.getIntegral();}
//            telemetry(names, values);
//        }
//
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);

    }

    public void  goToPositionSupreme (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError) throws InterruptedException {
        goToPositionSupreme(targetXPosition, targetYPosition, robotPower, gyroAngle, allowableDistanceError, pd.getP(), pd.getI(), pd.getD());
    }

    public void goToPositionSupreme(double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double p, double i , double d, double gyroPwr) {
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double circ = Math.PI * (3.93701);
        targetXPosition = mode ? targetXPosition : -targetXPosition;

        double originalP = pd.getP();
        double originalI = pd.getI();
        double originalD = pd.getD();

        pd.setPID(p, i, d);
        double angle;
        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
        double distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
        double snipPwr = robotPower;
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double target = distance;

        //for telemetry purpose
        double initialXPos = globalCoordinate.getGlobalX();
        double initialYPos = globalCoordinate.getGlobalY();
        double XOvershoot = 0;
        double YOvershoot = 0;

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pd.initPD(target);

        while (opModeIsActive() && distance > allowableDistanceError * COUNTS_PER_INCH) {
            double globalX = globalCoordinate.getGlobalX();
            double globalY = globalCoordinate.getGlobalY();
            double normalHeading = getNormalizedHeading();
            distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalX;
            distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalY;
            angle = Math.toDegrees(Math.atan2(distanceToYTarget, distanceToXTarget));
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            snipPwr = Math.abs(pd.actuator(target - distance)) * robotPower;

            fr.setPower(transformation((angle) + 90 - normalHeading, "fr") * snipPwr + gyro(gyroAngle, "fr") * (gyroPwr));
            fl.setPower(transformation((angle) + 90 - normalHeading, "fl") * snipPwr + gyro(gyroAngle, "fl") * (gyroPwr));
            br.setPower(transformation((angle) + 90 - normalHeading, "br") * snipPwr + gyro(gyroAngle, "br") * (gyroPwr));
            bl.setPower(transformation((angle) + 90 - normalHeading, "bl") * snipPwr + gyro(gyroAngle, "bl") * (gyroPwr));

            //overshoot
            if (globalY - targetYPosition < 0 && targetYPosition - initialYPos < 0) YOvershoot = Math.abs(globalY - targetYPosition) > YOvershoot ? Math.abs(globalY - targetYPosition) : YOvershoot;
            else if (globalY - targetYPosition > 0 && targetYPosition - initialYPos > 0) YOvershoot = Math.abs(globalY - targetYPosition) > YOvershoot ? Math.abs(globalY - targetYPosition) : YOvershoot;

            if (globalX - targetXPosition < 0 && targetXPosition - initialXPos < 0) XOvershoot = Math.abs(globalX - targetXPosition) > XOvershoot ? Math.abs(globalX - targetXPosition) : XOvershoot;
            else if (globalX - targetXPosition > 0 && targetXPosition - initialXPos > 0) XOvershoot = Math.abs(globalX - targetXPosition) > XOvershoot ? Math.abs(globalX - targetXPosition) : XOvershoot;

            Object[] names = {"fr", "fl", "br", "bl","p_contr", "i_contr", "d_contr", "dis_left_x", "dis_left_y"};
            Object[] values = {fr.getCurrentPosition(), fl.getCurrentPosition(), br.getCurrentPosition(), bl.getCurrentPosition(),pd.getPContrb(), pd.getIContrb(), pd.getDContrb(), distanceToXTarget / COUNTS_PER_INCH, distanceToYTarget / COUNTS_PER_INCH};

            telemetry(names, values);
        }
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        pd.setPID(originalP, originalI, originalD);
    }

    public void  goToPositionSupreme (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double p, double i , double d) throws InterruptedException{
        goToPositionSupreme(targetXPosition, targetYPosition, robotPower, gyroAngle, allowableDistanceError, p, i, d, 1);
    }

    public void  goToPositionSupreme (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double gyroPwr) throws InterruptedException{
        goToPositionSupreme(targetXPosition, targetYPosition, robotPower, gyroAngle, allowableDistanceError, pd.getP(), pd.getI(), pd.getD(), gyroPwr);
    }


    public void telemetry(Object[]... maps) {
        try {
            for (int number = 0; number < maps[0].length; number++) {
                telemetry.addData(maps[0][number].toString(), maps[1][number].toString());
            }
        } catch (IndexOutOfBoundsException e){
        }
        telemetry.addData("globalX", globalCoordinate.getGlobalX() / COUNTS_PER_INCH);
        telemetry.addData("globalY", globalCoordinate.getGlobalY() / COUNTS_PER_INCH);
        telemetry.addData("fr_pwr", fr.getPower());
        telemetry.addData("fl_pwr", fl.getPower());
        telemetry.addData("fr_pwr", fr.getCurrentPosition());
        telemetry.addData("fl_pwr", fl.getCurrentPosition());
        telemetry.addData("br_pwr", br.getCurrentPosition());
        telemetry.addData("bl_pwr", bl.getCurrentPosition());
        telemetry.update();
    }


    public void moveInchesX(double targetXPosition, double power, double angle, double gyroAngle, double allowableError) {
        double snipPwr;
        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
        double target = distanceToXTarget;
        pd.initPD(distanceToXTarget);
        while (Math.abs(distanceToXTarget) > allowableError * COUNTS_PER_INCH) {
            distanceToXTarget = target - globalCoordinate.getGlobalX();
            snipPwr = pd.actuator(target - distanceToXTarget) * power;
            double normalHeading = getNormalizedHeading();
            fr.setPower(transformation(2 * angle - gyroAngle, "fr") * snipPwr + gyro(gyroAngle, "fr"));
            fl.setPower(transformation(2 * angle - gyroAngle, "fl") * snipPwr + gyro(gyroAngle, "fl"));
            br.setPower(transformation(2 * angle - gyroAngle, "br") * snipPwr + gyro(gyroAngle, "br"));
            bl.setPower(transformation(2 * angle - gyroAngle, "bl") * snipPwr + gyro(gyroAngle, "bl"));
            Object[] names = {"X_left"};
            Object[] numbers = {distanceToXTarget};
            telemetry(names,numbers);
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void moveInchesY(double targetYPosition, double power, double angle, double gyroAngle, double allowableError) {
        double snipPwr;
        double distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
        double target = distanceToYTarget;
        pd.initPD(distanceToYTarget);
        while (Math.abs(distanceToYTarget) > allowableError * COUNTS_PER_INCH) {
            distanceToYTarget = target - globalCoordinate.getGlobalY();
            snipPwr = pd.actuator(target - distanceToYTarget) * power;
            double normalHeading = getNormalizedHeading();
            fr.setPower(transformation(2 * angle - gyroAngle, "fr") * snipPwr + gyro(gyroAngle, "fr"));
            fl.setPower(transformation(2 * angle - gyroAngle, "fl") * snipPwr + gyro(gyroAngle, "fl"));
            br.setPower(transformation(2 * angle - gyroAngle, "br") * snipPwr + gyro(gyroAngle, "br"));
            bl.setPower(transformation(2 * angle - gyroAngle, "bl") * snipPwr + gyro(gyroAngle, "bl"));
            Object[] names = {"Y_left"};
            Object[] numbers = {distanceToYTarget};
            telemetry(names,numbers);
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public double gyro(double angles, String motor) {
        //fr fl br bl
        double a = 100; //adjusting rate
        double normalHeading = getNormalizedHeading();
        //prevent the sharp change from 360 to 0 or vice versa
        if (angles >= 0 && angles <= 90) {
            if (normalHeading >= 270 && normalHeading <= 360) {
                angles += 360;
            }
        }
        if (angles >= 270 && angles <= 360) {
            if (normalHeading >= 0 && normalHeading <= 90) {
                angles += 360;
            }
        }
        double dif = angles - normalHeading;
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

    public void runWithoutEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnPID (double angle, Direction direction, double robotPower) {
        turnPID(angle, direction, robotPower, pd.getP(), pd.getI(), pd.getD());
    }

    public void turnPID (double angle, Direction direction, double robotPower, double p, double i, double d) {
        double power;
        globalCoordinate.stop();
        runWithoutEncoders();
        double angleLeft = Math.abs(getNormalizedHeading() - angle);
        double firstDiff = Math.abs(getNormalizedHeading() - angle);
        double error = 3.8;
        double originalP = pd.getP(), originalI = pd.getI(), originalD = pd.getD();
        pd.setPID(p, i, d);

        pd.initPD(angleLeft);
        if (direction == Direction.CLOCKWISE) {
            while (opModeIsActive() && angleLeft > error) {
                angleLeft = Math.abs(getNormalizedHeading() - angle);
                power = pd.actuator(firstDiff - angleLeft) * robotPower;
                fr.setPower(-power);
                fl.setPower(power);
                br.setPower(-power);
                bl.setPower(power);
                telemetry.addData("pd", power / robotPower);
                telemetry.addData("angle_left", angleLeft);
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && angleLeft > error) {
                angleLeft = Math.abs(getNormalizedHeading() - angle);
                power = pd.actuator(firstDiff - angleLeft) * robotPower;
                fr.setPower(power);
                fl.setPower(-power);
                br.setPower(power);
                bl.setPower(-power);
                telemetry.addData("pd", power / robotPower);
                telemetry.addData("pi", power / robotPower);
                telemetry.addData("angle_left", angleLeft);
                telemetry.update();
            }
        }
        stopAndResetMotor();

        pd.setPID(originalP, originalI, originalD);

        globalCoordinate.setLastEncoderCountLeft(0);
        globalCoordinate.setLastEncoderCountLeftBack(0);
        globalCoordinate.setLastEncoderCountRight(0);
        globalCoordinate.setLastEncoderCountRightBack(0);
        globalCoordinate.start();
        globalCoordinateThread.start();
    }

    public void stopAndResetMotor() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
