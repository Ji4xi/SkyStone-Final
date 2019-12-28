package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class DoubleServoDoubleMotor extends TeleOpMode {
    DcMotor iml;
    DcMotor imr;
    final double pwrMax = 0.5;
    double currentPwr;

    Servo rs;
    Servo ls;
    final double maxPosTiger = 0.1;
    final double maxPosKevin = 1;
    double currentPosMax = maxPosTiger;
    double currentServoPos = 0;
    final double minPos = 0;

    @Override
    public void init() {
//        iml = hardwareMap.dcMotor.get("iml");
//        iml.setDirection(DcMotorSimple.turnDirection.FORWARD);
//        imr = hardwareMap.dcMotor.get("imr");
//        imr.setDirection(DcMotorSimple.turnDirection.REVERSE);

        rs = hardwareMap.servo.get("rs");
        rs.setDirection(Servo.Direction.FORWARD);
        ls = hardwareMap.servo.get("ls");
        ls.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Motor_Pwr", currentPwr);
        telemetry.addData("Servo_Pos", rs.getPosition());
    }

    @Override
    public void updateData() {
//        updateSlides();
        updateServo();
    }

    public void updateIntake() {
        currentPwr = pwrMax * gamepad2.left_stick_y;
        iml.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
        imr.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
    }

    public void updateServo() {
        if (gamepad2.a) {
            currentPosMax = maxPosTiger;
        }
        if (gamepad2.b) {
            currentPosMax = maxPosKevin;
        }
        if (ls.getPosition() > currentPosMax) {
            currentServoPos = currentPosMax;
            ls.setPosition(currentPosMax);
        }
        if (rs.getPosition() > currentPosMax + 0.1) {
            currentServoPos = currentPosMax + 0.1;
            rs.setPosition(currentPosMax + 0.1);
        }
        if (rs.getPosition() < 0) {
            currentServoPos = 0;
            rs.setPosition(minPos);
            ls.setPosition(minPos);
        }
        if (gamepad2.dpad_up) {
            currentServoPos = rs.getPosition();
            currentServoPos += 0.003;
            rs.setPosition(currentServoPos);
            ls.setPosition(currentServoPos);
        }
        if (gamepad2.dpad_down) {
            currentServoPos = rs.getPosition();
            currentServoPos -= 0.003;
            rs.setPosition(currentServoPos);
            ls.setPosition(currentServoPos);
        }

    }
}
