package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
public class Claw extends TeleOpMode {
    Servo claw;
    double currentPosition;
    final double max = 1;
    final double min = 0;
    @Override
    public void init() {
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("currentPosition", claw.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad2.y) {
            currentPosition = 0;
        }
        if (gamepad1.a) {
            currentPosition = 0.5;
        }

        claw.setPosition(currentPosition);
    }
}
