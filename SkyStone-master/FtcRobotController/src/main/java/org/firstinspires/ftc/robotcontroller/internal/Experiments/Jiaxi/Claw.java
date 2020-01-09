package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
public class Claw extends TeleOpMode {
    Servo claw;
    Servo rotate;
    final double max = 1;
    final double min = 0;
    @Override
    public void init() {
        claw = hardwareMap.servo.get("grip.claw");
        claw.setDirection(Servo.Direction.REVERSE);
        rotate = hardwareMap.servo.get("rotate");
        rotate.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("currentRotationPosition", rotate.getPosition());
        telemetry.addData("currentPosition", claw.getPosition());
    }


    @Override
    public void updateData() {
        if (gamepad1.y) {
            claw.setPosition(0.85);
        }
        else if (gamepad1.a) {
            claw.setPosition(0.76);
        }

        if (gamepad1.x) {
            rotate.setPosition(0.16);
        }
        else if (gamepad1.b) {
            rotate.setPosition(0.9);
        }
    }
}
