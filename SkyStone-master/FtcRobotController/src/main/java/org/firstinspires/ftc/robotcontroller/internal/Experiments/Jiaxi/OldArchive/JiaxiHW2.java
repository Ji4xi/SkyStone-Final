package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
/**
 * Depending on whether or not the start button is pressed, there will be different methods
 * in setting the speed of the motor
 **/
@Disabled
@TeleOp
public class JiaxiHW2 extends TeleOpMode {
    DcMotor motor;
    double MOTOR_PWR_MAX = 0.9;
    double currentPower;
    boolean flag = false;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void stop(){
        super.stop();
    }

    @Override
    public void telemetry() {
        telemetry.addData("MOTOR_PWR", motor.getPower());
        telemetry.addData("mode", flag);
    }

    @Override
    public void updateData() {
        if (!flag) {
            currentPower = gamepad1.left_stick_y * MOTOR_PWR_MAX;
        }

        if (flag) {
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

        if (gamepad1.start) {
            flag = true;
        }
        if (gamepad1.back) {
            flag = false;
        }

        motor.setPower(currentPower);
    }
}
