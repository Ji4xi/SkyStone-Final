package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
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

        if(gamepad1.dpad_up) pos += 0.001;
        else if (gamepad1.dpad_down) pos -= 0.001;
        if (pos > 0.6) pos = 0.6;
        else if (pos < 0) pos = 0;

    }

    @Override
    public void telemetry() {
        telemetry.addData("pos", servo.getPower());
    }
}
