package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.SkyScraper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PIDMichael;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.RangerAuto;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.SkyStoneVuforiaAuto;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.RefCountedSwitchableCamera;

@Disabled
@Autonomous
public class SkyScraper extends SkyStoneVuforiaAuto {
    enum Direction {
        CLOCKWISE, COUNTERCLOCKWISE
    }
    protected BNO055IMU imu; //For detecting angles of rotation
    //driveTrain
    DcMotor lm;
    DcMotor rm;
    DcMotor rightIntake;
    DcMotor leftIntake;
    DcMotor lift;
    Servo rs;
    Servo ls;
    double driveTrainPwr = 0.45;
    final double intakePwr = 0.8;


    static final double COUNTS_PER_REVOLUTION = 1120; //40:1
    final double MAX_POS = 1;
    final double MIN_POS = 0;

    static final double DRIVE_GEAR_REDUCTION = 1/1.5; //Thiss is < 1.0 if geared up
    static final double WHEEL_DIAMETER_INCHES = 4.25;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    PIDMichael Rpid = new PIDMichael(0.001, Math.pow(10, -12), 10000); //experimentally found
    PIDMichael Lpid = new PIDMichael(0.001, Math.pow(10, -12), 10000); //experimentally found

    boolean mode = true;
    double foundSkyStoneAngle;
    @Override
    public void runOpMode() throws InterruptedException {
        //0.1 for turning, 0.8 for moving forward
        // don't get too close to 180 and -180
        initialize();
        waitForStart();

    }

    public void initialize() throws InterruptedException{
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //driveTrain
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");

        lm.setDirection(DcMotorSimple.Direction.FORWARD);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);

        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("claw");
        lm.setDirection(DcMotorSimple.Direction.FORWARD);

        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);

        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveFoundationServos(1);
        super.runOpMode();
    }

    public void turnAbsolutePID(double angle, Direction direction, double power) {
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        double integral = 0.5;
        double error = 5;
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (mode) {
            double angleLeft = Math.abs(getAbsoluteHeading() - angle);
            double firstDiff = Math.abs(getAbsoluteHeading() - angle);
            if (direction == Direction.CLOCKWISE) {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(-power);
                    lm.setPower(power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            } else {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(power);
                    lm.setPower(-power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            }
        } else {
            double angleLeft = Math.abs(getAbsoluteHeading() - (-angle));
            double firstDiff = Math.abs(getAbsoluteHeading() - (-angle));
            if (direction == Direction.COUNTERCLOCKWISE) {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - (-angle));
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(-power);
                    lm.setPower(power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            } else {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - (-angle));
                    if (power > integral) power = Range.clip(angleLeft / firstDiff, 0, 1) * power;
                    else power = integral;
                    rm.setPower(power);
                    lm.setPower(-power);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            }
        }
        stopMotor();
        lm.setDirection(DcMotorSimple.Direction.FORWARD);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void turnAbsolute(double angle, Direction direction, double maxpower) {
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        double integral = 0.5;
        double error = 5;
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (mode) {
            double angleLeft = Math.abs(getAbsoluteHeading() - angle);
            double firstDiff = Math.abs(getAbsoluteHeading() - angle);
            if (direction == Direction.CLOCKWISE) {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    rm.setPower(-maxpower);
                    lm.setPower(maxpower);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            } else {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - angle);
                    rm.setPower(maxpower);
                    lm.setPower(-maxpower);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            }
        } else {
            double angleLeft = Math.abs(getAbsoluteHeading() - (-angle));
            double firstDiff = Math.abs(getAbsoluteHeading() - (-angle));
            if (direction == Direction.COUNTERCLOCKWISE) {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - (-angle));
                    rm.setPower(-maxpower);
                    lm.setPower(maxpower);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            } else {
                while (opModeIsActive() && angleLeft > error) {
                    angleLeft = Math.abs(getAbsoluteHeading() - (-angle));
                    maxpower = 1;
                    rm.setPower(maxpower);
                    lm.setPower(-maxpower);
                    telemetry.addData("rm_pwr", rm.getPower());
                    telemetry.addData("lm_pwr", lm.getPower());
                    telemetry.addData("imu", getAbsoluteHeading());
                    telemetry.addData("angleLeft", angleLeft);
                    telemetry.addData("firstDiff", firstDiff);
                    telemetry.addData("clockwise", "true");
                    telemetry.update();
                }
            }
        }
        stopMotor();
        lm.setDirection(DcMotorSimple.Direction.FORWARD);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void moveForward(double rev) throws InterruptedException {
        double Rpower = 0.8, Lpower = 0.8;
        int target = (int) (rev * COUNTS_PER_REVOLUTION);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);


        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rpid.initPID(target, System.nanoTime());
        Lpid.initPID(target, System.nanoTime());
        double RsnipPwr, Rintegral = 0, LsnipPwr, Lintegral = 0;
        while(rm.isBusy() || lm.isBusy()) {
            RsnipPwr = Rpid.actuator(rm.getCurrentPosition(), System.nanoTime());
            LsnipPwr = Lpid.actuator(lm.getCurrentPosition(), System.nanoTime());
            if (Rpower < 0.5) Rintegral = 0.5 - Rpower;
            if (Lpower < 0.5) Lintegral = 0.5 - Lpower;
            Rpower = Range.clip(RsnipPwr, -1, 1) * driveTrainPwr;
            Lpower = Range.clip(LsnipPwr, -1, 1) * driveTrainPwr;
            if (rev < 0) {
                rm.setPower(-Rpower - Rintegral);
                lm.setPower(-Lpower - Lintegral);
            } else {
                rm.setPower(Rpower + Rintegral);
                lm.setPower(Lpower + Lintegral);
                //lm.setPower(Rpower + Rintegral);
            }

            telemetry.addData("Rsnippwr", RsnipPwr);
            telemetry.addData("Rintegral", Rintegral);
            telemetry.addData("Lsnippwr", LsnipPwr);
            telemetry.addData("Lintegral", Lintegral);
            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.addData("left target_left", target - lm.getCurrentPosition());
            telemetry.addData("right target_left", target - rm.getCurrentPosition());
            telemetry.update();
        }
        sleep(5000);
        stopMotor();
    }


//    public double moveForwardInches(double inches, boolean LookForSkyStone) throws InterruptedException {
//        double power = 0.08;
//        int target = (int) (inches * COUNTS_PER_INCH);
//        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rm.setTargetPosition(target);
//        lm.setTargetPosition(target);
//
//        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rm.setPower(power);
//        lm.setPower(power);
//
//        FoundSkyStoneAngle();
//        double position = rm.getCurrentPosition();
//        stopMotor();
//        return position;
//    }
    public void moveForwardInches(double inches, long sleep) throws InterruptedException {
        double power;
        double integral = 0;
        double snipPwr;
        int target = (int) (inches * COUNTS_PER_INCH);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);


        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rpid.initPID(target, System.nanoTime());
        while(rm.isBusy() || lm.isBusy()) {
            snipPwr = Rpid.actuator(rm.getCurrentPosition(), System.nanoTime());
            if (snipPwr < 0.5) integral = 0.5 - snipPwr;
            power = Range.clip(snipPwr + integral, -1, 1) * driveTrainPwr;
            if (inches < 0) {
                rm.setPower(-power);
                lm.setPower(-power);
            } else {
                rm.setPower(power);
                lm.setPower(power);
            }

            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("right target_left", target - rm.getCurrentPosition());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.addData("left target_left", target - lm.getCurrentPosition());
//            telemetry.addData("p", pid.p * pid.error);
//            telemetry.addData("i", pid.i * pid.integral);
//            telemetry.addData("d", pid.d * pid.differential);

            telemetry.update();
        }
        stopMotor();
        sleep(sleep, "go forward " + inches + " inches");
    }

    public void moveForwardInchesGyro(double inches, double targetHeading, long sleep) throws InterruptedException {

        double constant = 200;
        double integral = 0.2;
        double power;
        double error;

        int target = (int) (inches * COUNTS_PER_INCH);
        rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm.setTargetPosition(target);
        lm.setTargetPosition(target);

        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rpid.initPID(target, System.nanoTime());
        if (!mode) targetHeading *= -1;

        while(opModeIsActive() && rm.isBusy() && lm.isBusy()) {

            power = Range.clip(Rpid.actuator(rm.getCurrentPosition(), System.nanoTime()) + integral, -1, 1) * driveTrainPwr;
            error = getAbsoluteHeading() - targetHeading;



            //counter-intuitive functions
            if (inches < 0) {
                if (Math.abs(error) > 200) error = Math.abs(getAbsoluteHeading()) - Math.abs(targetHeading);
                if (error > 0) {
                    rm.setPower(-power + (error / constant));
                    lm.setPower(-power - (error / constant));
                } else {
                    rm.setPower(-power - (error / constant));//
                    lm.setPower(-power + (error / constant));//
                }
            } else {
                if (Math.abs(error) > 200) error = Math.abs(getAbsoluteHeading()) - Math.abs(targetHeading);
                if (error > 0) {
                    rm.setPower(power + (error / constant));
                    lm.setPower(power - (error / constant));
                } else {
                    rm.setPower(power + (error / constant));//
                    lm.setPower(power - (error / constant));//
                }
            }

            telemetry.addData("rm_pwr", rm.getPower());
            telemetry.addData("lm_pwr", lm.getPower());
            telemetry.addData("target_left rm", target - rm.getCurrentPosition());
            telemetry.addData("target_left lm", target - lm.getCurrentPosition());
            telemetry.addData("error", error);
            telemetry.addData("heading", getAbsoluteHeading());
            telemetry.update();
        }
        stopMotor();
        sleep(sleep, "go forward " + inches + " inches");
    }

    public void moveForwardInchesGyro(double inches, double targetHeading, long sleep, double power) throws InterruptedException{
        double temp = driveTrainPwr;
        driveTrainPwr = power;
        moveForwardInchesGyro(inches, targetHeading, sleep);
        driveTrainPwr = temp;
    }

//    public void claw(double power, double rev) {
//        claw.setTargetPosition((int) (rev * COUNTS_PER_REVOLUTION));
//        while (claw.isBusy()) {
//            claw.setPower(power);
//        }
//        stopMotor();
//    }

    public void moveForwardInchesGyro(double inches, double targetHeading) throws InterruptedException {
        moveForwardInchesGyro(inches, targetHeading, 0);
    }

    public void stopMotor() {
        rm.setPower(0);
        lm.setPower(0);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveFoundationServos(double position) {
        ls.setPosition(Range.clip(position, MIN_POS, MAX_POS));
        rs.setPosition(Range.clip(position, MIN_POS, MAX_POS));
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
