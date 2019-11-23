package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class CalvinHW1Numero5 extends TeleOpMode {

    DcMotor motor;
    double MOTOR_PWR_MAX = 0.8;
    double currentPower;

    String mode = "normal";
    Double multiplier = 0.5;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void telemetry() {
        telemetry.addData("MOTOR_PWR", motor.getPower());
//        telemetry.addData("mode", mode);
    }

    @Override
    public void updateData() {

        if (gamepad1.left_bumper) {
            mode = "fast";
            multiplier = 1.0;
        }
        else if (gamepad1.right_bumper) {
            mode = "slow";
            multiplier = 0.25;
        }
        else {
            mode = "normal";
            multiplier = 0.5;
        }

        currentPower = gamepad1.right_stick_x * multiplier * MOTOR_PWR_MAX;
        motor.setPower(currentPower);

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

}
