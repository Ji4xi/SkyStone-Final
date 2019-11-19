package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class ServoHwKevin extends TeleOpMode {
/*
Move servo position according to button pressed
 */
    Servo servo;
    final double MAX_POSITION = 0.5;
    final double MIN_POSITION = 0;
    double currentPosition;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("servo", servo.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad1.a ) {
            currentPosition = MIN_POSITION;
        }else if (gamepad1.x){
            currentPosition = 0.25;
        }

        if (gamepad1.dpad_up){
            currentPosition += 0.01;
        } else if (gamepad1.dpad_down){
            currentPosition -= 0.01;
        }

        if (currentPosition > MAX_POSITION) {
            currentPosition = MAX_POSITION;
        } else if (currentPosition < MIN_POSITION) {
            currentPosition = MIN_POSITION;
        }
        servo.setPosition(Range.clip(currentPosition, MIN_POSITION, MAX_POSITION));
    }
}
