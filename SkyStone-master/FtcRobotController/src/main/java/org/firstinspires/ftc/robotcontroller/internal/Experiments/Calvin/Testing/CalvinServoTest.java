package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class CalvinServoTest extends TeleOpMode {

    Servo left;
    Servo right;

    double MAX_POS = 1;
    double MIN_POS = 0;

    double currentPosition;

    @Override
    public void init() {
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");

        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("left", left.getPosition());
        telemetry.addData("right", right.getPosition());
    }

    @Override
    public void updateData() {

        if (gamepad1.x) {
            currentPosition = MAX_POS;
        } else if (gamepad1.y) {
            currentPosition = MIN_POS;
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

        left.setPosition(Range.clip(currentPosition, MIN_POS, MAX_POS));
        right.setPosition(Range.clip(currentPosition, MIN_POS, MAX_POS));

    }

}
