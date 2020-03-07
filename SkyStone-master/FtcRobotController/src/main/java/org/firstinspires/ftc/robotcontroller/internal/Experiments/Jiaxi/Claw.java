package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
@Disabled
public class Claw extends TeleOpMode {
    Servo claw;
    final double max = 1;
    final double min = 0;
    @Override
    public void init() {
        claw = hardwareMap.servo.get("grip.claw");
        claw.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("currentPosition", claw.getPosition());
    }


    @Override
    public void updateData() {
        if (gamepad1.y) {
            claw.setDirection(Servo.Direction.FORWARD);
            claw.setPosition(0.3);
        }
        else if (gamepad1.a) {
            claw.setDirection(Servo.Direction.REVERSE);
            claw.setPosition(0.9);
        }
    }
}
