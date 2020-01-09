package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
@TeleOp

public class JackBlack extends TeleOpMode {

    Servo top;
    Servo bot;

    @Override
    public void init() {
        top = hardwareMap.servo.get("top");
        bot = hardwareMap.servo.get("bot");

        top.setDirection(Servo.Direction.FORWARD);
        bot.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("tos", top.getPosition());
        telemetry.addData("bos", bot.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad1.y) {
            top.setPosition(0);
        }

        if (gamepad1.a) {
            bot.setPosition(0);
        }
    }


}