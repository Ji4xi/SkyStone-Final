package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class CalvinServoIntake extends TeleOpMode {

    Servo servo;

    double MAX_POS = 0.5;
    double MIN_POS = 0.1;

    double currentPosition;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(MAX_POS);
        currentPosition = MAX_POS;
    }

    @Override
    public void telemetry() {
        telemetry.addData("servo", servo.getPosition());
    }

    @Override
    public void updateData() {

        if (gamepad1.x) {
            currentPosition = MAX_POS;
        } else if (gamepad1.y) {
            currentPosition = MIN_POS;
        } else if (gamepad1.a) {
            currentPosition = 0.45;
        }

        if (gamepad1.dpad_up) {
            currentPosition += 0.01;
        } else if (gamepad1.dpad_down) {
            currentPosition -= 0.01;
        }

        //bounds
        if (currentPosition > MAX_POS) {
            currentPosition = MAX_POS;
        } else if (currentPosition < MIN_POS) {
            currentPosition = MIN_POS;
        }

        servo.setPosition(Range.clip(currentPosition, MIN_POS, MAX_POS));

    }

}
