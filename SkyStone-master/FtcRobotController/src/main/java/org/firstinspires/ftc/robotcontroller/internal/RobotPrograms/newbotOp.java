package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.MecanumXDrive;

//@Disabled
@TeleOp
public class newbotOp extends TeleOpMode {

    MecanumXDrive mecanumXDrive = new MecanumXDrive();
    DcMotor rightIntake;
    DcMotor leftIntake;
    Servo claw;
    Servo rs;
    Servo ls;

    final double intakePwr = 0.4;

    @Override
    public void init() {

        mecanumXDrive.syncOpMode(gamepad1, telemetry, hardwareMap);
        mecanumXDrive.init("fr","fl","br","bl");

        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");

        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.FORWARD);
        ls.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void telemetry() {
        mecanumXDrive.telemetry();
        telemetry.addData("rightIntake_pwr", rightIntake.getPower());
        telemetry.addData("leftIntake_pwr", leftIntake.getPower());
        telemetry.addData("lift_pos", claw.getPosition());
        telemetry.addData("rs_ pos", rs.getPosition());
        telemetry.addData("ls_pos", ls.getPosition());
    }

    @Override
    public void updateData() {
        updateDriveTrain();
        updateIntake();
        updateClaw();
        updateFoundation();
    }

    public void updateFoundation() {
        if (gamepad1.a) {
            rs.setPosition(0);
            ls.setPosition(0);
        } else if (gamepad1.y) {
            rs.setPosition(1);
            ls.setPosition(1);
        }
    }

    public void updateDriveTrain() {
        mecanumXDrive.update();
    }

    public void updateIntake() {
        if(gamepad2.left_bumper) {
            rightIntake.setPower(intakePwr);
            leftIntake.setPower(intakePwr);
        }
        else if(gamepad2.right_bumper) {
            rightIntake.setPower(-0.3);
            leftIntake.setPower(-0.3);
        }
        else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
    }

    public void updateClaw() {
        if (gamepad2.a) {
            claw.setPosition(0.5);
        } else if (gamepad2.y) {
            claw.setPosition(0);
        }
    }
}
