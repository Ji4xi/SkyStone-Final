package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import android.media.MediaDrm;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
@TeleOp
@Disabled
public class JackBlack extends TeleOpMode {

    Servo top;
    Servo bot;
    double topPos = 0, botPos = 1;

    @Override
    public void init() {
        top = hardwareMap.servo.get("top");
        bot = hardwareMap.servo.get("bot");

        top.setDirection(Servo.Direction.FORWARD);
        bot.setDirection(Servo.Direction.FORWARD);

        bot.setPosition(0.1669);
        top.setPosition(0.855);

    }

    @Override
    public void telemetry() {
        telemetry.addData("tos", top.getPosition());
        telemetry.addData("bos", bot.getPosition());
    }

    @Override
    public void updateData() {

        if (gamepad1.x) {
            topPos = 0.035;
        }
        else if (gamepad1.y) {
            topPos = 0.4;
        }

        if (gamepad1.a) {
            botPos = 0;
        }
        else if (gamepad1.b) {
            botPos = 0.3;
        }

        if (gamepad1.dpad_up) botPos += 0.001;
        else if (gamepad1.dpad_down) botPos -= 0.001;
        else if (gamepad1.dpad_left) topPos += 0.001;
        else if (gamepad1.dpad_right) topPos -= 0.001;

        if (topPos > 1) topPos = 1;
        else if (topPos < 0) topPos = 0;

        if (botPos > 1) botPos = 1;
        else if (botPos < 0) botPos = 0;

        top.setPosition(topPos);
        bot.setPosition(botPos);
    }


}
