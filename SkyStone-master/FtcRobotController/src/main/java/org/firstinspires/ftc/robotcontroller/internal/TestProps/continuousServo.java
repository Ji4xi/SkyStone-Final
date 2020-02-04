package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class continuousServo extends TeleOpMode {
    CRServo servo;
    double pos;

    @Override
    public void init() {
       servo = hardwareMap.crservo.get("servo");
       servo.setDirection(CRServo.Direction.REVERSE);
       servo.setPower(0);
    }

    @Override
    public void updateData() {
        servo.setPower(gamepad1.left_stick_y);
    }

    @Override
    public void telemetry() {
        telemetry.addData("pos", servo.getPower());
    }
}
