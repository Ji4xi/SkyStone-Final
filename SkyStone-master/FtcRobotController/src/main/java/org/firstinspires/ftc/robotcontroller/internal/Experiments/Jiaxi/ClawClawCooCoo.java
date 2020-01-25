package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

import java.nio.channels.OverlappingFileLockException;

@TeleOp

public class ClawClawCooCoo extends TeleOpMode {
    Servo clawServo;
    double currentPosition = 0.5;
    final double max = 0;
    final double min = 0;
    @Override
    public void init() {
        clawServo = hardwareMap.servo.get("clawServo");
        clawServo.setDirection(Servo.Direction.REVERSE);
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
        telemetry.addData("currentPosition", clawServo.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad1.y) {
            currentPosition = 0.7;
            clawServo.setPosition(currentPosition);
        }
        if (gamepad1.x) {
            currentPosition = 1;
            clawServo.setPosition(currentPosition);
        }
    }
}
