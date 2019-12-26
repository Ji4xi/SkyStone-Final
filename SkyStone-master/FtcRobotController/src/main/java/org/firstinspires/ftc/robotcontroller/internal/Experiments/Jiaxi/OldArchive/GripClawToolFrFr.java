package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
@Disabled
public class GripClawToolFrFr extends TeleOpMode {
    Servo s360;
    Servo s180;
    Servo railreach;
    double currentPos;
    final double max180 = 1;
    final double minPos = 0;
    final double max360 = 2;
    @Override
    public void init() {
        s180 = hardwareMap.servo.get("s180");
        s360 = hardwareMap.servo.get("s360");
        s180.setDirection(Servo.Direction.FORWARD);
        s360.setDirection(Servo.Direction.FORWARD);
        railreach = hardwareMap.servo.get("railreach");
        railreach.setDirection(Servo.Direction.FORWARD);
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
        telemetry.addData("realreach", railreach.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad2.y) {
            s360.setPosition(0.4);
        }
        if (gamepad2.a) {
            s360.setPosition(0.8);
        }
        if (gamepad2.x) {
            s180.setPosition(0);
        }
        if (gamepad2.b) {
            s180.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            railreach.setPosition(1);
        }  else if (gamepad2.dpad_up) {
            railreach.setPosition(0.79);
        }

    }
}
