package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
@Disabled
@TeleOp
public class JiaxiTest1 extends TeleOpMode {
    double MOTOR_PWR_MAX = 0.9;
    double currentPower;
    boolean activated = false;
    DcMotor motor;


    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        telemetry.addData("MOTOR_PWR", motor.getPower());
        telemetry.addData("ACTIVATED", activated);
    }

    @Override
    public void updateData() {
        if (!activated) {
            currentPower = currentPower * MOTOR_PWR_MAX;
        }
        if (activated) {
            if (gamepad1.x) {
                currentPower = 0.8;
            }
            if (gamepad1.y) {
                currentPower = 0.5;
            }
            if (gamepad1.dpad_up) {
                currentPower += 0.01;
            }
            if (gamepad1.dpad_down) {
                currentPower -= 0.01;
            }
        }
        motor.setPower(currentPower);
        if (gamepad1.start) {
            activated = true;
        }
        if (gamepad1.back) {
            activated = false;
        }
    }
}
