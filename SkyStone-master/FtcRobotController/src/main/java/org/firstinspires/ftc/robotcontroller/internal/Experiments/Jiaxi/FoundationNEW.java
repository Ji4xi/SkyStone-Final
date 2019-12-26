package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
public class FoundationNEW extends TeleOpMode{
    Servo max;
    Servo michelle;
    final double maxPos = 1;
    final double minPos = 0;
    @Override
    public void init() {
        max = hardwareMap.servo.get("right");
        max.setDirection(Servo.Direction.FORWARD);
        michelle = hardwareMap.servo.get("left");
        michelle.setDirection(Servo.Direction.FORWARD);
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

    }

    @Override
    public void updateData() {
        if (gamepad1.y) {
             max.setPosition(minPos);
             michelle.setPosition(maxPos);
        }
        else if (gamepad1.x) {
            max.setPosition(0.5);
            michelle.setPosition(0.5);
        }
    }
}
