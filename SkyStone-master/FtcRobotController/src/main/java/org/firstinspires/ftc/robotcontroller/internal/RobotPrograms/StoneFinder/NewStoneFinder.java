package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.PD;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class NewStoneFinder extends opencvSkystoneDetector {

    protected boolean mode = true;

    protected BNO055IMU imu;//For detecting angles of rotation
    protected enum Mode {OPEN, CLOSE}
    protected enum Side {LEFT, RIGHT}
    protected enum GAGE {FORWARD, REVERSE}

    protected Servo grip;
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
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public enum Direction {CLOCKWISE, COUNTERCLOCKWISE}

    protected GlobalCoordinate globalCoordinate;

    //Field Variables
    protected final double TILE_INCH = 22.75;
    protected final double SKYSTONE_INCH = 8;

    protected Thread globalCoordinateThread;
    public PD pd = new PD(0.001, 0, 0); //I: 0.001/ D: 0.0002/2.5 P?0.00067 0.00074 .00083//march 1st: 0.00064, 0.0000004, 0.000027

    public double skystoneAngle;
    public double maxPwr = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
    }

    /**
     * initalize hardware
     * @throws InterruptedException
     */
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

        grip = hardwareMap.servo.get("grip");
        grip.setDirection(Servo.Direction.FORWARD);
        grip.setPosition(0.65);

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

    /**
     * move with default PID and gyro power
     */
    public void  goToPositionSupreme (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError) throws InterruptedException {
        goToPositionSupreme(targetXPosition, targetYPosition, robotPower, gyroAngle, allowableDistanceError, pd.getP(), pd.getI(), pd.getD());
    }

    /**
     * move with PID and Gyro
     * @param targetXPosition target absolute X in inches
     * @param targetYPosition target absolute Y in inches
     * @param robotPower movement power
     * @param gyroAngle correct angle for imu
     * @param allowableDistanceError acceptable margin of error in inches
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     * @param gyroPwr gyro adjustment power
     */
    public void goToPositionSupreme(double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double p, double i , double d, double gyroPwr) {
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        targetXPosition = mode ? targetXPosition : -targetXPosition;

        double originalP = pd.getP();
        double originalI = pd.getI();
        double originalD = pd.getD();

        pd.setPID(p, i, d);

        double angle;
        double distanceToXTarget = targetXPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalX();
        double distanceToYTarget = targetYPosition * COUNTS_PER_INCH - globalCoordinate.getGlobalY();
        double snipPwr;
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

    /**
     * move with default gyro power
     */
    public void  goToPositionSupreme (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double p, double i , double d) throws InterruptedException{
        goToPositionSupreme(targetXPosition, targetYPosition, robotPower, gyroAngle, allowableDistanceError, p, i, d, 1);
    }

    /**
     * move with default PID
     */
    public void  goToPositionSupreme (double targetXPosition, double targetYPosition, double robotPower, double gyroAngle, double allowableDistanceError, double gyroPwr) throws InterruptedException{
        goToPositionSupreme(targetXPosition, targetYPosition, robotPower, gyroAngle, allowableDistanceError, pd.getP(), pd.getI(), pd.getD(), gyroPwr);
    }

    /**
     * updates telemetry
     * @param maps arrays with extra telemetry values
     */
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

    /**
     * returns gyro adjustment
     * @param angles correct angle for imu
     * @param motor drive motor
     * @return calculated gyro adjustment
     */
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

    /**
     * returns transformation for mecanum wheels
     * @param angles angle in radians
     * @param motor drive motor
     * @return transformed angle
     */
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
     * updates current velocity of robot
     */
    public void updateVelocity() {
        double lastCount;
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

    /**
     * set drive motors to run without encoders
     */
    public void runWithoutEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * turn with default PID
     */
    public void turnPID (double angle, Direction direction, double robotPower) {
        turnPID(angle, direction, robotPower, pd.getP(), pd.getI(), pd.getD());
    }

    /**
     * turn using PID
     * @param angle angle to turn to in degrees
     * @param direction clockwise or counterclockwise
     * @param robotPower turn power
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     * @param error acceptable margin of error in degrees
     */
    public void turnPID (double angle, Direction direction, double robotPower, double p, double i, double d, double error) {
        double power;
        globalCoordinate.stop();
        runWithoutEncoders();
        double angleLeft = Math.abs(getNormalizedHeading() - angle);
        double firstDiff = Math.abs(getNormalizedHeading() - angle);
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

    /**
     *turn with default error
     */
    public void turnPID (double angle, Direction direction, double robotPower, double p, double i, double d) {
        turnPID(angle, direction, robotPower, p, i, d, 3.8);
    }

    /**
     * resets drive motors
     */
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

    /**
     * gets heading of imu
     * @return heading from imu
     */
    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * gets heading of imu based on unit circle
     * @return normalized heading
     */
    public double getNormalizedHeading() {
        return (getAbsoluteHeading() + 450) % 360;
    }

    /**
     * moves foundation servos
     * @param position Servos up or down
     * @param sleep Sleep time after in milliseconds
     * @throws InterruptedException
     */
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
