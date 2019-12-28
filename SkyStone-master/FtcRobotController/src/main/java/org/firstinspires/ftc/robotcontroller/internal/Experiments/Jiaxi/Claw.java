package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class Claw extends TeleOpMode {
    Servo rc;
    Servo linearServo;
    double rcPos;
    double linearServoPos = 0;
    final double MAX = 1;
    final double MIN = 0;
    final double rcMax = 0.4;
    @Override
    public void init() {
        rc = hardwareMap.servo.get("rc");
        linearServo = hardwareMap.servo.get("linearServo");
        rc.setDirection(Servo.Direction.FORWARD);
        linearServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void telemetry() {
        telemetry.addData("xrail servo position", linearServo.getPosition());
        telemetry.addData("claw position", rc.getPosition());
        telemetry.addData("other xrail servo position", linearServoPos);
        telemetry.addData("other claw postion", rcPos);
        telemetry.update();
    }

    @Override
    public void updateData() {
//        if (gamepad1.dpad_up) {
//            rcPos += 0.002;
//        } else if (gamepad1.dpad_down) {
//            rcPos -= 0.002;
//        }
//
////        else if (gamepad1.dpad_left) {
////            linearServoPos += 0.002;
////        } else if (gamepad1.dpad_right) {
////            linearServoPos -= 0.002;
////        }
        if (gamepad1.y) {
            rcPos = 0.1;
        }
        else if (gamepad1.a) {
            rcPos = 0;
        }

        //bounds
        if (rcPos > rcMax) {
            rcPos = rcMax;
        } else if (rcPos < MIN) {
            rcPos = MIN;
        }
//        if (linearServoPos > MAX) {
//            linearServoPos = MAX;
//        } else if (linearServoPos < MIN) {
//            linearServoPos = MIN;
//        }

        rc.setPosition(rcPos);
//        linearServo.setPosition(linearServoPos);
    }
}
