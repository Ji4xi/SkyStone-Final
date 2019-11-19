package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class JiaxiClaw extends TeleOpMode {
    Servo leftServo;
    Servo rightServo;
    
    double MAX_POS = 1;
    double MIN_POS = 0;
    double currentPosition;
    
    @Override
    public void init() {
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("leftServo", leftServo.getPosition());
        telemetry.addData("rightServo", rightServo.getPosition());
    }

    @Override
    public void updateData() {

        if (gamepad1.x) {
            currentPosition = MAX_POS;
        }
        else if (gamepad1.y) {
            currentPosition = MIN_POS;
        }
        if (gamepad1.dpad_up) {
            currentPosition += 0.01;
        }
        else if (gamepad1.dpad_down) {
            currentPosition -= 0.01;
        }
        //bounds
        if (currentPosition > MAX_POS){
            currentPosition = MAX_POS;
        }
        if (currentPosition < MIN_POS){
            currentPosition = MIN_POS;
        }
        leftServo.setPosition(Range.clip(currentPosition, MIN_POS, MAX_POS));
        rightServo.setPosition(Range.clip(currentPosition, MIN_POS, MAX_POS));
    }
}
