package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
//@Disabled
@TeleOp
public class KevinservoTest extends TeleOpMode{
    Servo servo;

    double MAX_POSITION = 1;
    double MIN_POSITION = 0;
    double currentPosition;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo.setDirection(Servo.Direction.REVERSE);
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
        telemetry.addData("servo", servo.getPosition());
    }
//bounds
    @Override
    public void updateData() {
        if (gamepad1.a) {
            currentPosition = MAX_POSITION;
            currentPosition = 0.5;
            currentPosition = 0.1;
            currentPosition = 0.7;
        }else if (gamepad1.b){
            currentPosition = MIN_POSITION;
        }
        if (gamepad1.dpad_up) {
            currentPosition += 0.01;
        }else if (gamepad1.dpad_down){
            currentPosition -= 0.01;
        }
        if (currentPosition > MAX_POSITION){
            currentPosition = MAX_POSITION;
        }
        if (currentPosition < MIN_POSITION){
            currentPosition = MIN_POSITION;
        }
        servo.setPosition(currentPosition);
    }
}