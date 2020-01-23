package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
@Disabled
public class KevinClaw extends TeleOpMode {
    Servo servo1;
    Servo servo2;
    
    double MAX_POSITION = 1;
    double MIN_POSITION = 0;
    double currentPosition;

    @Override
    public void init() {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
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
        telemetry.addData("servo1" , servo1.getPosition());
        telemetry.addData("servo2" , servo2.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad1.dpad_up){
            currentPosition += 0.01;
        }
        else if (gamepad1.dpad_down){
            currentPosition -= 0.01;
        }
        
        if (gamepad1.a){
            currentPosition = MAX_POSITION;
        }
        else if (gamepad1.b){
            currentPosition = MIN_POSITION;
        }
        //bounds
        if (currentPosition < 0){
            currentPosition = MIN_POSITION;
        }
        if (currentPosition > 1) {
            currentPosition = MAX_POSITION;
        }
        servo1.setPosition(Range.clip(currentPosition, MIN_POSITION, MAX_POSITION));
        servo2.setPosition(Range.clip(currentPosition, MIN_POSITION, MAX_POSITION));
    }
}
