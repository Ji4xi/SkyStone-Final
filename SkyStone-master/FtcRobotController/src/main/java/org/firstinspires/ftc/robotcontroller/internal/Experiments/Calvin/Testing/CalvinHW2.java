package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class CalvinHW2 extends TeleOpMode {

    DcMotor motor;

    double MOTOR_PWR_MAX = 0.9;
    double currentPower;

    String mode = "button";

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void telemetry() {
        telemetry.addData("MOTOR_PWR", motor.getPower());
        telemetry.addData("mode", mode);
    }

    @Override
    public void updateData() {

        if (gamepad1.left_bumper) {
            mode = "button";
        }
        else if (gamepad1.right_bumper) {
            mode = "stick";
        }

        if (mode.equals("button")) {

            if (gamepad1.x) {
                currentPower = 0.8;
            }
            else if (gamepad1.y) {
                currentPower = 0.5;
            }

            if (gamepad1.dpad_up && currentPower < 1) {
                currentPower += 0.01;
            }
            else if (gamepad1.dpad_down && currentPower > -1) {
                currentPower -= 0.01;
            }

        }
        else {
            currentPower = gamepad1.right_stick_y;
        }

        motor.setPower(currentPower * MOTOR_PWR_MAX);

    }

}
