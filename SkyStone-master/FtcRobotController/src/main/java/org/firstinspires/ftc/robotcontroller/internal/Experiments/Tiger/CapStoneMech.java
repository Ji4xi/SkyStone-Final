package org.firstinspires.ftc.robotcontroller.internal.Experiments.Tiger;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class CapStoneMech extends TeleOpMode {

    Servo mech;

    @Override
    public void init() {
        mech = hardwareMap.servo.get("servo");
        mech.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void telemetry() {
        telemetry.addData("pos", mech.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad1.y) {
            mech.setPosition(0.3);
        } else if (gamepad1.a) {
            mech.setPosition(0.8);
        }
    }

}
