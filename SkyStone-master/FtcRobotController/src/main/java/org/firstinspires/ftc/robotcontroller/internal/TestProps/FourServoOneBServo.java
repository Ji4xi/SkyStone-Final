package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class FourServoOneBServo extends TeleOpMode {
    Servo rs;
    Servo ls;
    Servo brs;
    Servo bls;
    Servo bs;
    final double maxPosDSD = 1;
    final double minPosDSD = 0;
    double currentServoPos;
    double bigCurrent;
    @Override
    public void init() {
        rs = hardwareMap.servo.get("rs");
        rs.setDirection(Servo.Direction.REVERSE);
        rs.setPosition(minPosDSD);
        brs = hardwareMap.servo.get("brs");
        brs.setDirection(Servo.Direction.REVERSE);
        brs.setPosition(minPosDSD);

        ls = hardwareMap.servo.get("ls");
        ls.setDirection(Servo.Direction.FORWARD);
        ls.setPosition(minPosDSD);
        bls = hardwareMap.servo.get("bls");
        bls.setDirection(Servo.Direction.FORWARD);
        bls.setPosition(minPosDSD);

        bs = hardwareMap.servo.get("bs");
        bs.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("rsServo_Pos", rs.getPosition());
        telemetry.addData("lsSDServo_Pos", ls.getPosition());
        telemetry.addData("brsSDServo_Pos", brs.getPosition());
        telemetry.addData("blsDServo_Pos", bls.getPosition());
        telemetry.addData("BsServo_Pos", bs.getPosition());
    }

    @Override
    public void updateData() {
        if (ls.getPosition() > maxPosDSD) {
            currentServoPos = maxPosDSD;
            ls.setPosition(maxPosDSD);
        }
        if (rs.getPosition() > maxPosDSD) {
            currentServoPos = maxPosDSD;
            rs.setPosition(currentServoPos);
        }
        if (brs.getPosition() > maxPosDSD) {
            currentServoPos = maxPosDSD;
            brs.setPosition(currentServoPos);
        }
        if (bls.getPosition() > maxPosDSD) {
            currentServoPos = maxPosDSD;
            bls.setPosition(currentServoPos);
        }
        if (rs.getPosition() < 0) {
            currentServoPos = 0;
            rs.setPosition(minPosDSD);
            ls.setPosition(minPosDSD);
            brs.setPosition(minPosDSD);
            brs.setPosition(minPosDSD);
        }
        if (gamepad2.dpad_up) {
            currentServoPos = rs.getPosition();
            currentServoPos += 0.003;
            rs.setPosition(currentServoPos);
            ls.setPosition(currentServoPos);
            brs.setPosition(currentServoPos);
            bls.setPosition(currentServoPos);
        }
        if (gamepad2.dpad_down) {
            currentServoPos = rs.getPosition();
            currentServoPos -= 0.003;
            rs.setPosition(currentServoPos);
            ls.setPosition(currentServoPos);
            brs.setPosition(currentServoPos);
            bls.setPosition(currentServoPos);
        }
        if(gamepad2.y) {
            bigCurrent = bs.getPosition();
            bigCurrent += 0.003;
            bs.setPosition(Range.clip(bigCurrent, 0, 1));
            if (bigCurrent > 1) {
                bigCurrent = 1;
            } else if (bigCurrent < 0) {
                bigCurrent = 0;
            }
        } else if (gamepad2.a) {
            bigCurrent = bs.getPosition();
            bigCurrent -= 0.003;
            bs.setPosition(Range.clip(bigCurrent, 0, 1));
            if (bigCurrent > 1) {
                bigCurrent = 1;
            } else if (bigCurrent < 0) {
                bigCurrent = 0;
            }
        }

    }
}
