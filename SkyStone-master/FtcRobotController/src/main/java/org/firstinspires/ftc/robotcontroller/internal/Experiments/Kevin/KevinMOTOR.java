package org.firstinspires.ftc.robotcontroller.internal.Experiments.Kevin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
@Disabled
@TeleOp
public class KevinMOTOR extends TeleOpMode{
    DcMotor motor;
    double MOTOR_PWR_MAX = 0.8;
    double currentPower;


    @Override
    public void init() {
         motor = hardwareMap.dcMotor.get("dcMotor");
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
        telemetry.addData("dcMotor", motor.getPower());
    }

    @Override
    public void updateData() {

        if (gamepad1.a){
            currentPower = 0.7;
        }

        if (gamepad1.b){
            currentPower = 0.4;
        }
        currentPower = gamepad1.left_stick_y * MOTOR_PWR_MAX;
        motor.setPower(currentPower);
    }
}

