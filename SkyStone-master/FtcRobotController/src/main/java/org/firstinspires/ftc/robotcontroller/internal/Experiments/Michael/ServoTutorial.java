package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
/**
* program to turn a servo 0 to 90 degree
*/
public class ServoTutorial extends TeleOpMode {
    Servo servo;
    double maxPos = 0.5;
    double minPos = 0;
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
        if (gamepad1.y) {
            currentPosition = maxPos;
        } else if (gamepad1.x) {
            currentPosition = minPos;
        }

        //adjust position using dpad
        if (gamepad1.dpad_up) {
            currentPosition += 0.01;
        } else if (gamepad1.dpad_down) {
            currentPosition -= 0.01;
        }

        //bound
        if (currentPosition > maxPos) {
            currentPosition = maxPos;
        }
        if (currentPosition < minPos) {
            currentPosition = minPos;
        }

        servo.setPosition(Range.clip(currentPosition, minPos, maxPos));
    }
}
