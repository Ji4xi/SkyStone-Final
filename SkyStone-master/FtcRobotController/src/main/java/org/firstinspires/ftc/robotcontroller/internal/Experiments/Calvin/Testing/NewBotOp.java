package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class NewBotOp extends TeleOpMode {

    DcMotor rightIntake;
    DcMotor leftIntake;

    Servo s;

    final double intakePwr = 0.8;
    final double liftPwr = 0.4;

    @Override
    public void init() {
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);


        s = hardwareMap.servo.get("s");
        s.setDirection(Servo.Direction.REVERSE);

        s.setPosition(1);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Intake Speed", rightIntake.getPower());
        telemetry.addData("servo postition", s.getPosition());
    }

    @Override
    public void updateData() {
        updateIntake();
        updateServo();
    }

    public void updateServo() {
        if (gamepad2.a) {
            s.setPosition(0);
            s.setPosition(0);
        } else if (gamepad2.y) {
            s.setPosition(0.5);
            s.setPosition(0.5);
        }
    }

    public void updateIntake() {
        if(gamepad2.right_bumper) {
            rightIntake.setPower(intakePwr);
            leftIntake.setPower(intakePwr);
        }
        else if(gamepad2.left_bumper) {
            rightIntake.setPower(-intakePwr);
            leftIntake.setPower(-intakePwr);
        }
        else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }
    }

}
