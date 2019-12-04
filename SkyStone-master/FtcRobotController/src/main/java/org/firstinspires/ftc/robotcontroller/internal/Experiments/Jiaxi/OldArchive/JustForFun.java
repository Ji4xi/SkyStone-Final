package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class JustForFun extends TeleOpMode {
    DcMotor motor;
    double MOTOR_PWR_MAX = 0.8;
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
        telemetry.addData("MOTOR_PWR", motor.getPower());
    }

    @Override
    public void updateData() {
//        motor.setPower(0.5); set the power of motor a constant of 0.5 for the entire time
        if (gamepad1.x) {
            motor.setPower(0.3);
        }
        if (gamepad2.y) {
            motor.setPower(1);
        }
        currentPower = gamepad1.left_stick_y * MOTOR_PWR_MAX;
        motor.setPower(currentPower);
    }

}
