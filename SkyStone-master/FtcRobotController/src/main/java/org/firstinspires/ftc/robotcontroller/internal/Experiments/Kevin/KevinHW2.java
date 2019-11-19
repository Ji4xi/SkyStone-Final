package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
@Disabled
@TeleOp
public class KevinHW2 extends TeleOpMode {
    DcMotor motor;
    double MAX_MOTOR_PWR = 0.9;
    double currentPower;

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
        telemetry.addData("motor", motor.getPower());
    }

    @Override
    public void updateData() {
        if (gamepad1.x){
            currentPower = 0.8;
        }
        if (gamepad1.y){
            currentPower = 0.5;
        }
        if (gamepad1.right_bumper){
            currentPower = gamepad1.right_stick_y * MAX_MOTOR_PWR;
        }
        if (gamepad1.dpad_up){
            currentPower += 0.01;
        }
        if (gamepad1.dpad_down) {
            currentPower -= 0.01;
        }
    }
}

