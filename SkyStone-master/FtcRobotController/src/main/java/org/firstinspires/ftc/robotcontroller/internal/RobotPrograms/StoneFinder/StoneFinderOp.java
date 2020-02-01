package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.MecanumXDrive;

//@Disabled
@TeleOp
public class StoneFinderOp extends TeleOpMode {
    MecanumXDrive mecanumXDrive = new MecanumXDrive();

    DcMotor rightIntake;
    DcMotor leftIntake;

    Servo hook;

    Servo rs;
    Servo ls;

    Servo topClaw;
    Servo botClaw;

    Servo grip;
    Servo rotate;

    Servo extender;



    DcMotor rightLift;
    DcMotor leftLift;


    final double intakePwr = 0.7;
    final double maxLiftPwr = 0.3;
    double currentLiftPwr = 0;

    final double slidePwr = 0.9;

    final double COUNTS_PER_REVOLUTION = 288;
    final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / 6.69291;
    final double COUNTS_PER_BRICK = COUNTS_PER_INCH * 5;

    final double SERVO_TICK_PER_REV = 1 / 7.5;
    double extenderPos = 0;
    final double EXTENDER_MAX_POS = 0.8;
    final double EXTENDER_MIN_POS = 0.3;



    @Override
    public void init() {
        mecanumXDrive.syncOpMode(gamepad1, telemetry, hardwareMap);
        mecanumXDrive.init("fr","fl","br","bl");

        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        hook = hardwareMap.servo.get("hook");
        hook.setDirection(Servo.Direction.FORWARD);

        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);

        rightLift = hardwareMap.dcMotor.get("rightLift");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);



        topClaw = hardwareMap.servo.get("top");
        botClaw = hardwareMap.servo.get("bot");
        topClaw.setDirection(Servo.Direction.REVERSE);
        botClaw.setDirection(Servo.Direction.REVERSE);


        topClaw.setPosition(0);
        botClaw.setPosition(0.35);

        extender = hardwareMap.servo.get("extender");
        extender.setDirection(Servo.Direction.FORWARD);
        extender.setPosition(EXTENDER_MIN_POS);

        grip = hardwareMap.servo.get("grip");
        rotate = hardwareMap.servo.get("rotate");
        grip.setDirection(Servo.Direction.FORWARD);
        rotate.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void telemetry() {
        mecanumXDrive.telemetry();
        telemetry.addData("rightIntake_pwr", rightIntake.getPower());
        telemetry.addData("leftIntake_pwr", leftIntake.getPower());
        telemetry.addData("lift_pos", hook.getPosition());
        telemetry.addData("rs_ pos", rs.getPosition());
        telemetry.addData("ls_pos", ls.getPosition());
        telemetry.addData("leftSlide_pwr", leftLift.getPower());
        telemetry.addData("rightSlide_pwr", rightLift.getPower());
        //telemetry.addData("tos", topClaw.getPosition());
        //telemetry.addData("bos", botClaw.getPosition());
        telemetry.addData("gripper pos", grip.getPosition());
        telemetry.addData("rotator pos", rotate.getPosition());
        telemetry.addData("extender pwr", extender.getPosition());
        telemetry.addData("claw top", topClaw.getPosition());
        telemetry.addData("claw bottom", botClaw.getPosition());
    }

    @Override
    public void updateData() {
        updateDriveTrain();
        updateIntake();
        updateHook();
        updateFoundation();
        updateLift();
        //updateClaw();
        updateExtender();
        updateGrip();
        updateDrive();
    }

    //public void updateClaw() {
    //
    //}
    public void updateDrive() {

    }

    public void updateFoundation() {
        if (gamepad1.left_bumper) {
            rs.setPosition(0);
            ls.setPosition(0);
        } else if (gamepad1.right_bumper) {
            rs.setPosition(0.5);
            ls.setPosition(0.5);
        }
    }

    public void updateDriveTrain() {
        mecanumXDrive.update();
        if (gamepad1.x) mecanumXDrive.flipMechanic(true);
        else if (gamepad1.b) mecanumXDrive.flipMechanic(false);
    }

    public void updateIntake() {
        if(gamepad2.left_bumper) {
            rightIntake.setPower(intakePwr);
            leftIntake.setPower(intakePwr);
        }
        else if(gamepad2.right_bumper) {
            rightIntake.setPower(-0.45);
            leftIntake.setPower(-0.45);
        }
        else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
    }

    public void updateHook() {
        if (gamepad2.a) {
            hook.setPosition(0.8);
        } else if (gamepad2.y) {
            hook.setPosition(0.3);
        }
        currentLiftPwr = gamepad2.left_stick_y * maxLiftPwr;
        leftLift.setPower(Range.clip(currentLiftPwr, - maxLiftPwr, maxLiftPwr));
        rightLift.setPower(Range.clip(currentLiftPwr, - maxLiftPwr, maxLiftPwr));
    }

    public void updateLift() {
        double leftSlidePwr = gamepad2.left_stick_y * slidePwr;
        double rightSlidePwr = gamepad2.left_stick_y * slidePwr;

        leftSlidePwr = gamepad2.left_stick_y > 0 ? 0.35 : leftSlidePwr;
        rightSlidePwr = gamepad2.left_stick_y > 0 ? 0.35 : rightSlidePwr;


        if (gamepad2.x) {
            double nearest = COUNTS_PER_BRICK * Math.round((leftLift.getCurrentPosition() + rightLift.getCurrentPosition()) / 2 / COUNTS_PER_BRICK);

            leftSlidePwr =  slidePwr * (nearest - leftLift.getCurrentPosition())/COUNTS_PER_BRICK;
            rightSlidePwr = slidePwr * (nearest - rightLift.getCurrentPosition())/COUNTS_PER_BRICK;

        }

        if (gamepad2.left_stick_y == 0) {
            leftSlidePwr = -0.1565;
            rightSlidePwr = -0.1565;
        }

        leftLift.setPower(leftSlidePwr);
        rightLift.setPower(rightSlidePwr);
    }

    public void updateExtender() {
        double mode;
        mode = gamepad2.right_stick_y;
        if (mode < 0) {
            extenderPos += 0.002;
        }
        else if (mode > 0) {
            extenderPos -= 0.002;
        }

//        if (gamepad2.dpad_up) {
//            extenderPos += 0.001;
//        } else if (gamepad2.dpad_down) {
//            extenderPos -= 0.001;
//        }

        //bounds
        if (extenderPos > EXTENDER_MAX_POS) {
            extenderPos = EXTENDER_MAX_POS;
        } else if (extenderPos < EXTENDER_MIN_POS) {
            extenderPos = EXTENDER_MIN_POS;
        }

        double diff1 = Math.abs(extenderPos - EXTENDER_MAX_POS);
        double diff2 = Math.abs(extenderPos - EXTENDER_MIN_POS);
        if (gamepad2.b && diff1 < diff2) {
            extenderPos = EXTENDER_MAX_POS;
        } else if (gamepad2.b && diff1 > diff2) {
            extenderPos = EXTENDER_MIN_POS;
        }

        if (gamepad2.dpad_up) extenderPos = EXTENDER_MAX_POS;
        else if (gamepad2.dpad_down) extenderPos = EXTENDER_MIN_POS;
        extender.setPosition(Range.clip(extenderPos, EXTENDER_MIN_POS, EXTENDER_MAX_POS));
    }

    public void updateGrip() {
        if (gamepad1.y) {
            grip.setPosition(0.55);
        }
        else if (gamepad1.a) {
            grip.setPosition(0.11); //0.76
        }

        if (gamepad1.dpad_down) {
            rotate.setPosition(0.08);
        }
        else if (gamepad1.dpad_left) {
            rotate.setPosition(0.465);
        }
        else if (gamepad1.dpad_up) {
            rotate.setPosition(0.85);
        }
    }

}