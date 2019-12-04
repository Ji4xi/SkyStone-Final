package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
@Disabled
public class Foundation extends TeleOpMode {

    Servo rf;
    Servo lf;


    @Override
    public void init() {
        rf = hardwareMap.servo.get("rf");
        lf = hardwareMap.servo.get("lf");
        rf.setDirection(Servo.Direction.FORWARD);
        lf.setDirection(Servo.Direction.REVERSE);
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
            rf.setPosition(1);
            lf.setPosition(1);
        }
        if (gamepad1.a) {
            rf.setPosition(0);
            lf.setPosition(0);
        }
    }
}
