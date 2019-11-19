package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class OneServoDump extends TeleOpMode {
    Servo os;
    final double maxPos = 1;
    final double midPos = 0.625;
    final double minPos = 0.25;
    @Override
    public void init() {
        os = hardwareMap.servo.get("rs");
        os.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("OServo_Pos", os.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad2.y) {
            os.setPosition(maxPos);
        }
        if (gamepad2.x) {
            os.setPosition(midPos);
        }
        if (gamepad2.a) {
            os.setPosition(minPos);
        }
    }
}
