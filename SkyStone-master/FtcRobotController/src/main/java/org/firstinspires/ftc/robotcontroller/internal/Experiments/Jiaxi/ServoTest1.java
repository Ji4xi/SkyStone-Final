package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.onbotjava.StandardResponses;
import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class ServoTest1 extends TeleOpMode {
    Servo servo;

    double MAX_POS = 0.5;
    double MIN_POS = 0;

    double currentPosition;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo.setDirection(Servo.Direction.REVERSE);
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
        telemetry.addData("servo", currentPosition);

    }

    @Override
    public void updateData() {
        if (gamepad1.x) {
            currentPosition = MAX_POS;
        }
        if (gamepad1.y) {
            currentPosition = MIN_POS;
        }
        if (gamepad1.dpad_up) {
            currentPosition += 0.01;
        }
        if (gamepad1.dpad_down) {
            currentPosition -= 0.01;
        }

        //bounds
        if (currentPosition > MAX_POS) {
            currentPosition = MAX_POS;
        }
        if (currentPosition < MIN_POS) {
            currentPosition = MIN_POS;
        }
        servo.setPosition(currentPosition);
    }
}