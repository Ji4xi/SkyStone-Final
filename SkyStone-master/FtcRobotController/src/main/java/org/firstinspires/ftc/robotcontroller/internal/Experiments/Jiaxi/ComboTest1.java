package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class ComboTest1 extends TeleOpMode {
    DcMotor motor;
    Servo servo;
    double MOTOR_MAX_PWR = 0.9;
    double currentPower;
    double MAX_POS = 1;
    double MIN_POS = 0;
    double currentPosition;
    boolean switchFunctions = false;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        telemetry.addData("motor", currentPower);
        telemetry.addData("servo", currentPosition);
    }

    @Override
    public void updateData() {
        if (!switchFunctions) {
            currentPower = currentPower * MOTOR_MAX_PWR;
        }
        if (switchFunctions) {
            if (gamepad1.dpad_up) {
                currentPosition += 0.01;
            }
            if (gamepad1.dpad_down) {
                currentPosition -= 0.01;
            }
        }
        motor.setPower(currentPower);
        servo.setPosition(currentPosition);
        if (gamepad1.start) {
            switchFunctions = true;
        }
        if (gamepad1.back) {
            switchFunctions = false;
        }
    }
}
