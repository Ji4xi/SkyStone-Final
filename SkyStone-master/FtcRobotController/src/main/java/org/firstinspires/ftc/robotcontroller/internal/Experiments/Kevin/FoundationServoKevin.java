package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class FoundationServoKevin extends TeleOpMode {
    Servo rs;
    Servo ls;
    @Override
    public void init() {
        rs = hardwareMap.servo.get("rs");
        ls = hardwareMap.servo.get("ls");
        rs.setDirection(Servo.Direction.REVERSE);
        ls.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void updateData() {
        if (gamepad2.a){
            ls.setPosition(0);
            rs.setPosition(0);
        }
        else if (gamepad2.y){
            ls.setPosition(1);
            rs.setPosition(1);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("ls", ls.getPosition());
        telemetry.addData("rs", rs.getPosition());
    }
}
