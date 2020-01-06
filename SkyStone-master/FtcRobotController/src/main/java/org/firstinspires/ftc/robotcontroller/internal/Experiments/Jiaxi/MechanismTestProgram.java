package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class MechanismTestProgram extends TeleOpMode {
    Servo grip;
    Servo extension;
    final double max = 1;
    final double min = 0;
    double currentPosition;
    @Override
    public void init() {
        grip = hardwareMap.servo.get("grip");
        extension = hardwareMap.servo.get("extension");
        grip.setDirection(Servo.Direction.FORWARD);
        extension.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("Grip Position", grip.getPosition());
        telemetry.addData("Extension Position", extension.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad1.left_bumper) {
            grip.setPosition(0.5);
        }
        if (gamepad1.right_bumper) {
            grip.setPosition(0);
        }

        currentPosition = gamepad1.left_stick_y;
        if (currentPosition > max) {
            currentPosition = max;
        }
        if (currentPosition < min) {
            currentPosition = min;
        }
        extension.setPosition(currentPosition);

    }
}
