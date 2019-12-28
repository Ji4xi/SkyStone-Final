package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp

public class Lift extends TeleOpMode {
    final double drumDiameter = 2;
    double drumCirc = Math.PI * (drumDiameter);
    final double brickHeight = 5;
    final double oneDrumRotation = 0;
    double currentPower = 0;

    //Define Motors
    DcMotor rightLiftMotor;
    DcMotor leftLiftMotor;

    @Override
    public void init() {
        rightLiftMotor = hardwareMap.dcMotor.get("rm");
        leftLiftMotor = hardwareMap.dcMotor.get("lm");
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        telemetry.addData("RightDrumPosition", rightLiftMotor.getCurrentPosition());
        telemetry.addData("LeftDrumPosition", leftLiftMotor.getCurrentPosition());
        telemetry.addData("Average Drum Position", (rightLiftMotor.getCurrentPosition() + leftLiftMotor.getCurrentPosition()) / 2);
    }

    @Override
    public void updateData() {
        if (gamepad1.y) {
            currentPower += 0.01;
        }
        else if (gamepad1.a) {
            currentPower -= 0.01;
        }

        else {
            currentPower = 0;
        }

        rightLiftMotor.setPower(currentPower);
        leftLiftMotor.setPower(currentPower);
    }
}
