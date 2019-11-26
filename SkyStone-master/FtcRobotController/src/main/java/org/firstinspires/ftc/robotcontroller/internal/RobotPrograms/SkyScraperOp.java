package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.ArcadeDrive;

//@Disabled
@TeleOp
public class SkyScraperOp extends TeleOpMode {
    DcMotor lm;
    DcMotor rm;
    DcMotor rightIntake;
    DcMotor leftIntake;
    DcMotor lift;
    Servo rs;
    Servo ls;
    final double driveTrainPwr = 0.8;
    final double intakePwr = 0.8;
    final double liftPwr = 0.4;

    @Override
    public void init() {
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");

        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setDirection(DcMotorSimple.Direction.FORWARD);

        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);

        rs.setPosition(1);
        ls.setPosition(1);
    }

    @Override
    public void telemetry() {
        telemetry.addData("left power", lm.getPower());
        telemetry.addData("right power", rm.getPower());
        telemetry.addData("Intake Speed", rightIntake.getPower());
        telemetry.addData("lift power", lift.getPower());
        telemetry.addData("rs_ pos", rs.getPosition());
        telemetry.addData("ls_pos", ls.getPosition());
    }

    @Override
    public void updateData() {
        updateDriveTrain();
        updateIntake();
        updateClaw();
        updateLift();
    }

    public void updateClaw() {
        if (gamepad2.a) {
            rs.setPosition(0);
            ls.setPosition(0);
        } else if (gamepad2.y) {
            rs.setPosition(1);
            ls.setPosition(1);
        }
    }
    public void updateDriveTrain() {
        double lmPwr, rmPwr;
        lmPwr = (gamepad1.left_stick_y - gamepad1.right_stick_x) * driveTrainPwr;
        rmPwr = (gamepad1.left_stick_y + gamepad1.right_stick_x) * driveTrainPwr;
        lm.setPower(lmPwr);
        rm.setPower(rmPwr);
    }

    public void updateIntake() {
        if(gamepad2.left_bumper) {
            rightIntake.setPower(intakePwr);
            leftIntake.setPower(intakePwr);
        }
        else if(gamepad2.right_bumper) {
            rightIntake.setPower(-intakePwr);
            leftIntake.setPower(-intakePwr);
        }
        else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
    }

    public void updateLift() {
        double lp;
        lp = gamepad2.left_stick_y > 0 ? 1 : -1;
        lp = gamepad2.left_stick_y == 0 ? 0 : lp;
        lift.setPower(lp*liftPwr);
    }
}
